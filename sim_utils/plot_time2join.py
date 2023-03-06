import matplotlib.pyplot as plt
import os

# Constants
strs = "WS SM:Bootstrap Done"

# Variables
t_join = dict()

for i in range(1, 255):
    path=f"/tmp/n{i}_"
    if os.path.exists(path):
        with open (path , 'rb') as f:
            lines = f.readlines()
            t_start = lines[0].decode('utf-8').split(' ')[0]
            t_end = 0
            for line  in lines:
                #if all(w in strs for w in str(line)):
                if strs in str(line):
                    #print(line)
                    t_end = line.decode('utf-8').split(' ')[0]
                    print(f"n{i}_ {t_start} {t_end}")
                    break
        if (t_end): t_join[f"n{i}"] = int(t_end)-int(t_start)

# plot t_join
print(t_join)
plt.plot(t_join.keys(), t_join.values(), '-o')
plt.xlabel('Node')
plt.ylabel('Time (s)')
plt.title('Time to join (line topology)')
plt.grid(linestyle='--')
plt.yticks(range(100, max(t_join.values())+60, 100))
plt.savefig('time2join.pdf')
#plt.show()
