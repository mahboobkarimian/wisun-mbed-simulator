#include <stdint.h>
#include <stdbool.h>
#include <poll.h>
#include <sys/un.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/socket.h>
#include <sys/resource.h>
#include "common/log.h"
#include "common/utils.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>

#include "nodes_infos.h"

#define MAX_NODES 4096

struct node_infos *nodes_infos_flat_map;

struct ctxt {
    struct sockaddr_un addr;
    uint64_t node_graph[MAX_NODES][MAX_NODES / 64];
};

static int increase_limit_fd()
{
    struct rlimit rlimit;

    getrlimit(RLIMIT_NOFILE, &rlimit);
    DEBUG("Increase file descriptors limit from %ld to %ld",
          rlimit.rlim_cur, rlimit.rlim_max);
    rlimit.rlim_cur = rlimit.rlim_max;
    setrlimit(RLIMIT_NOFILE, &rlimit);
    return rlimit.rlim_cur;
}

static int bitmap_get(int shift, uint64_t *in, int size)
{
    int word_nr = shift / 64;
    int bit_nr = shift % 64;

    BUG_ON(word_nr >= size);
    return !!(in[word_nr] & (1ULL << bit_nr));
}

static int bitmap_set(int shift, uint64_t *out, int size)
{
    int word_nr = shift / 64;
    int bit_nr = shift % 64;

    if (word_nr >= size)
        return -1;
    out[word_nr] |= 1ULL << bit_nr;
    return 0;
}

static int bitmap_clr(int shift, uint64_t *out, int size)
{
    int word_nr = shift / 64;
    int bit_nr = shift % 64;

    if (word_nr >= size)
        return -1;
    out[word_nr] &= ~(1ULL << bit_nr);
    return 0;
}

static int bitmap_parse(char *str, uint64_t *out, int size)
{
    char *range;
    char *endptr;
    unsigned long cur, end;

    char *str_copy = (char *)malloc(strlen(str));
    if (str_copy == NULL) {
        return -1;
    }
    strcpy(str_copy, str);

    char *indexes_part = strtok(str_copy, ":");

    if (indexes_part == NULL) {
        // No RSSI has been specified
        strcpy(str_copy, str);
    }

    memset(out, 0, size * sizeof(uint64_t));
    range = strtok(str_copy, ",");
    do {
        cur = strtoul(range, &endptr, 0);
        if (*endptr == '-') {
            range = endptr + 1;
            end = strtol(range, &endptr, 0);
        } else {
            end = cur;
        }
        if (*endptr != '\0') {
            free(str_copy);
            return -1;
        }
        if (cur > end) {
            free(str_copy);
            return -1;
        }
        for (; cur <= end; cur++)
            if (bitmap_set(cur, out, size) < 0) {
                free(str_copy);
                return -1;
            }
    } while ((range = strtok(NULL, ",")));

    free(str_copy);
    return 0;
}

static int rssi_map_parse_for_one_group(char *str)
{
    char *range;
    char *endptr;
    int nodes_in_group[MAX_NODES] = {-1};
    memset(nodes_in_group, -1, sizeof(nodes_in_group));
    int nodes_in_group_idx = 0;

    char *str_copy = (char *)malloc(strlen(str));
    if (str_copy == NULL) {
        return -1;
    }
    strcpy(str_copy, str);

    char *rest = str_copy;
    char *indexes_part, *rssi_part;
    float rssi = 0.0;
    indexes_part = __strtok_r(rest, ":", &rest);


    if (indexes_part == NULL || strstr(str, ":") == NULL) {
        // No RSSI has been specified
        free(str_copy);
        return 0;
    }

    rssi_part = __strtok_r(rest, ":", &rest);

    if (rssi_part != NULL) {
        rssi = (float)atof(rssi_part);
    }

    strcpy(str_copy, str);
    rest = str_copy;
    indexes_part = __strtok_r(rest, ":", &rest);

    // Map the RSSI
    range = strtok(indexes_part, ",");
    unsigned long start_node = strtoul(range, &endptr, 0);
    unsigned long end_node;
    if (*endptr == '-') {
        range = ++endptr;
        end_node = strtoul(range, &endptr, 0);

        if (end_node > start_node) {
            for (int i = start_node + 1; i <= end_node; i++) {
                for (int j = start_node + 1; j <= end_node; j++) {
                    if (i != j) {
                        (nodes_infos_flat_map + i * sizeof(nodes_infos_flat_map) + j)->rssi = rssi;
                        (nodes_infos_flat_map + j * sizeof(nodes_infos_flat_map) + i)->rssi = rssi;
                    }
                }
            }
        }
    } else {
        nodes_in_group[nodes_in_group_idx++] = start_node;
        while ((range = strtok(NULL, ","))) {
            unsigned long cur_node = strtoul(range, &endptr, 0);
            nodes_in_group[nodes_in_group_idx++] = cur_node;
        }

        int i = 0, j = 0;
        while (nodes_in_group[i] != -1) {
            while (nodes_in_group[j] != -1) {
                if (i != j) {
                    (nodes_infos_flat_map + nodes_in_group[i] * sizeof(nodes_infos_flat_map) * MAX_NODES + nodes_in_group[j])->rssi = rssi;
                    (nodes_infos_flat_map + nodes_in_group[j] * sizeof(nodes_infos_flat_map) * MAX_NODES + nodes_in_group[i])->rssi = rssi;
                }
                j++;
            }
            j = 0;
            i++;
        }
    }

    free(str_copy);

    return 0;
}

static void graph_apply_mask(uint64_t node_graph[MAX_NODES][MAX_NODES / 64], uint64_t mask[MAX_NODES / 64])
{
    int i, j;

    for (i = 0; i < MAX_NODES; i++)
        if (bitmap_get(i, mask, MAX_NODES / 64))
            for (j = 0; j < MAX_NODES; j++)
                if (bitmap_get(j, mask, MAX_NODES / 64))
                    bitmap_set(j, node_graph[i], MAX_NODES / 64);
}

static int graph_get_num_nodes(struct ctxt *ctxt)
{
    int max = 0;
    int i, j;

    for (i = 0; i < MAX_NODES; i++)
        for (j = 0; j < MAX_NODES; j++)
            if (bitmap_get(j, ctxt->node_graph[i], MAX_NODES / 64))
                max = i;
    return max + 1;
}

static void graph_dump(struct ctxt *ctxt)
{
    int max = graph_get_num_nodes(ctxt);
    int i, j;

    printf("Topology:\n");
    for (i = 0; i < max; i++) {
        printf("%02x ", i);
        for (j = 0; j < max; j++) {
            char bf[10];
            sprintf(bf, "%02x|", j);
            printf("%s", bitmap_get(j, ctxt->node_graph[i], MAX_NODES / 64) ? bf : "--|");
        }
        printf("\n");
    }

    printf("\nRSSI map:\n");
    for (i = 0; i < max; i++) {
        if (i == 0) {
            printf("%*d ", 11, i);
        } else {
            printf("%*d ", 5, i);
        }

    }
    printf("\n");
    for (i = 0; i < max; i++) {
        printf("%*d ", 5, i);
        for (j = 0; j < max; j++) {
            printf("%*.*g ", 5, 6, (nodes_infos_flat_map + i * sizeof(nodes_infos_flat_map) * MAX_NODES + j)->rssi);
        }
        printf("\n");
    }
}

static void graph_dump_to_file(struct ctxt *ctxt, char * file_path)
{
    int max = graph_get_num_nodes(ctxt);
    int i, j;

    int fd = open(file_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        perror("open");
        exit(-1);
    }

    char *ptr = (char*)calloc(1024*1028, sizeof(char));
    if (ptr == NULL) {
        perror("memalloc");
        exit(-1);
    }
    char *bptr = ptr;

    for (i = 0; i < max; i++) {
        for (j = 0; j < max; j++) {
            if(bitmap_get(j, ctxt->node_graph[i], MAX_NODES / 64)) {
                sprintf(bptr, "%04d,%04d\n", i, j);
                bptr = bptr + 4 + 4 + 2; // MAX_NODES is 4096 which is 4 chars * 2 plus 2 for a "\n" and a ","
            }
        }
    }
    char buf[15] = {0};
    snprintf(buf, 15, "%04d\n", max);
    write(fd, &buf, 5);
    write(fd, ptr, bptr - ptr);
    close(fd);
    free(ptr);
}

void print_help(FILE *stream, int exit_code) {
    fprintf(stream, "broadcast server to create networks of wshwsim\n");
    exit(exit_code);
}

void parse_commandline(struct ctxt *ctxt, int argc, char *argv[])
{
    const char *opts_short = "hlf::g:";
    static const struct option opts_long[] = {
        { "group", required_argument, 0,  'g' },
        { "fpath", optional_argument, 0, 'f'},
        { "dump",  no_argument,       0,  'l' },
        { "help",  no_argument,       0,  'h' },
        { 0,       0,                 0,   0  }
    };
    uint64_t mask[MAX_NODES / 64];
    bool dump = false, has_filter = false;
    bool dump2file = false;
    char dump_path[4096];
    int opt, i, ret;

    while ((opt = getopt_long(argc, argv, opts_short, opts_long, NULL)) != -1) {
        switch (opt) {
            case 'g':
                ret = bitmap_parse(optarg, mask, MAX_NODES / 64);
                ret = rssi_map_parse_for_one_group(optarg);
                FATAL_ON(ret, 1, "Bad mask: %s", optarg);
                graph_apply_mask(ctxt->node_graph, mask);
                has_filter = true;
                break;
            case 'f':
               if (optarg == NULL && optind < argc
                    && argv[optind][0] != '-')
                {
                    optarg = argv[optind++];
                }
                if (optarg == NULL)
                {
                    sprintf(dump_path, "%s", "/tmp/graph.txt");
                } else {
                    strncpy(dump_path, optarg, sizeof(dump_path)-2);
                    dump_path[sizeof(dump_path)-1] = '\n';
                }
                    dump2file = true;
                break;
            case 'l':
                dump = true;
                break;
            case 'h':
                print_help(stdout, 0);
                break;
            case '?':
                print_help(stderr, 1);
                break;
            default:
                break;
        }
    }
    if (!has_filter)
        memset(ctxt->node_graph, 0xFF, sizeof(ctxt->node_graph));
    for (i = 0; i < MAX_NODES; i++)
        bitmap_clr(i, ctxt->node_graph[i], MAX_NODES / 64);
    if (dump) {
        FATAL_ON(!has_filter, 1, "No graph to dump");
        graph_dump(ctxt);
        if (dump2file) {
            graph_dump_to_file(ctxt, dump_path);
        }
    }
    if (optind >= argc)
        FATAL(1, "Expected argument: socket path");
    if (optind + 1 < argc)
        FATAL(1, "Too many arguments argument: %s", argv[optind + 1]);
    FATAL_ON(strlen(argv[optind]) >= sizeof(ctxt->addr.sun_path), 1);
    strcpy(ctxt->addr.sun_path, argv[optind]);
}

static void broadcast(uint64_t *node_graph, struct pollfd *fds, int fds_len, void *buf, int buf_len)
{
    int j;
    int ret;

    for (j = 0; j < fds_len; j++) {
        if (fds[j].fd >= 0 && bitmap_get(j, node_graph, MAX_NODES / 64)) {
            ret = write(fds[j].fd, buf, buf_len);
            FATAL_ON(ret != buf_len, 1, "write: %m");
        }
    }
}

bool stop = false;
void signal_handler(int signal) {
    if (signal == SIGINT) {
        stop = true;
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);

    // Shared memory for CCA
    const char* shm_name = "/wssimcca";
    const int shm_size = 4096*3; // 4096 nodes, 1 byte state + 2 bytes channel

    // Create shared memory
    int shm_fd = shm_open(shm_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd, shm_size);

    // Map shared memory
    void* ptr = mmap(0, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    memset(ptr, 0, shm_size);

    // Shared memory for nodes and edges information
    const char* shm_infos_name = "/wssimnodesinfos";
    const int shm_infos_size = MAX_NODES*MAX_NODES*sizeof(struct node_infos);

    // Create shared memory
    int shm_infos_fd = shm_open(shm_infos_name, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_infos_fd, shm_infos_size);

    // Map shared memory
    void* infos_ptr = mmap(0, shm_infos_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_infos_fd, 0);

    nodes_infos_flat_map = (struct node_infos *)infos_ptr;
    for (int i = 0; i < MAX_NODES; i++) {
        for (int j = 0; j < MAX_NODES; j++) {
            (nodes_infos_flat_map + i * sizeof(nodes_infos_flat_map) + j)->rssi = 0.0;
        }
    }

    char buf[4096];
    int i;
    int on = 1;
    int ret, len;
    int fd_limit;
    struct pollfd fds[MAX_NODES + 1] = { };
    struct ctxt ctxt = {
        .addr.sun_family = AF_UNIX
    };

    fd_limit = min(increase_limit_fd(), ARRAY_SIZE(fds));
    parse_commandline(&ctxt, argc, argv);
    for (i = 0; i < fd_limit; i++)
        fds[i].fd = -1;

    fds[0].events = POLLIN;
    fds[0].fd = socket(AF_UNIX, SOCK_SEQPACKET, 0); // use SOCK_SEQPACKET or SOCK_STREAM
    FATAL_ON(fds[0].fd < 0, 1, "socket: %s: %m", ctxt.addr.sun_path);
    ret = setsockopt(fds[0].fd, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof(on));
    FATAL_ON(ret < 0, 1, "setsockopt: %s: %m", ctxt.addr.sun_path);
    ret = bind(fds[0].fd, (struct sockaddr *)&ctxt.addr, sizeof(ctxt.addr));
    FATAL_ON(ret < 0, 1, "bind: %s: %m", ctxt.addr.sun_path);
    ret = listen(fds[0].fd, 4096);
    FATAL_ON(ret < 0, 1, "listen: %s: %m", ctxt.addr.sun_path);

    while (stop == false) {
        ret = poll(fds, fd_limit, -1);
        //FATAL_ON(ret < 0, 1, "poll: %m");
        if (ret < 0)
        {
            WARN("Poll error");
            break;
        }
        if (fds[0].revents) {
            for (i = 0; i < fd_limit; i++) {
                if (fds[i].fd == -1) {
                    fds[i].events = POLLIN;
                    fds[i].fd = accept(fds[0].fd, NULL, NULL);
                    DEBUG("Connect fd %d", fds[i].fd);
                    FATAL_ON(fds[i].fd < 0, 1, "accept: %m");
                    break;
                }
            }
            if (i == fd_limit)
                FATAL(1, "can't accept new node %d %d", i, fd_limit);
        }
        for (i = 1; i < fd_limit; i++) {
            if (fds[i].revents) {
                len = read(fds[i].fd, buf, sizeof(buf));
                if (len < 1) {
                    DEBUG("Disconnect fd %d", fds[i].fd);
                    close(fds[i].fd);
                    fds[i].fd = -1;
                    fds[i].events = 0;
                } else {
                    broadcast(ctxt.node_graph[i - 1], fds + 1, fd_limit - 1, buf, len);
                    if (len == 6 && buf[0] == 'x' && buf[1] == 'x') {
                        len = read(fds[i].fd, buf, sizeof(buf));
                        broadcast(ctxt.node_graph[i - 1], fds + 1, fd_limit - 1, buf, len);
                    }
                }
            }
        }
    }

    // Graceful teardown
    DEBUG("Graceful teardown");
    munmap(ptr, shm_size);
    munmap(infos_ptr, shm_infos_size);
    close(shm_fd);
    close(shm_infos_fd);
    shm_unlink(shm_name);
    shm_unlink(shm_infos_name);
}


