# Sends to cmd_thrust

if [ $# -eq 0 ]; then
    echo "Usage: $0 <model_name>"
    exit 1
fi

if [ "$1" = "bluerov2" ]; then
   ign topic -t /model/$1/joint/thruster5_joint/cmd_thrust -m ignition.msgs.Double -p 'data: 0.30'
   ign topic -t /model/$1/joint/thruster6_joint/cmd_thrust -m ignition.msgs.Double -p 'data: 0.30'
   
elif [ "$1" = "bluerov2_heavy" ]; then
   ign topic -t /model/$1/joint/thruster5_joint/cmd_thrust -m ignition.msgs.Double -p 'data: 0.30'
   ign topic -t /model/$1/joint/thruster6_joint/cmd_thrust -m ignition.msgs.Double -p 'data: 0.30'
   ign topic -t /model/$1/joint/thruster7_joint/cmd_thrust -m ignition.msgs.Double -p 'data: 0.30'
   ign topic -t /model/$1/joint/thruster8_joint/cmd_thrust -m ignition.msgs.Double -p 'data: 0.30'
   
else
   echo "Unsupported model provided. Please choose from either 'bluerov2' or 'bluerov2_heavy'"
   exit 1
fi