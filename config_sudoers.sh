#!/bin/sh

# Create sudoers config rule to run wsbrd as root without password
CURRENT_USER="$(whoami)"
CURRENT_PATH="$(pwd)"
SUDOERS_RULE_FILE="/etc/sudoers.d/wisun_sim_wsbrd"
SUDOERS_RULE="${CURRENT_USER} ALL=(ALL) NOPASSWD: ${CURRENT_PATH}/wsbrd"

if [ ! -f "$SUDOERS_RULE_FILE" ];then
  echo "Generate sudoers rule"
  echo $SUDOERS_RULE | sudo tee $SUDOERS_RULE_FILE
else
  echo "Sudoers rule file already exists"
fi
