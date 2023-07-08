#!/bin/bash

# Initialize variables to store total scores
total_mapping_score=0
total_planning_score=0

for i in {1..10}
do
    # Start the server in a new terminal with a unique title
    terminal_title="startC4-$i"
    gnome-terminal --title="$terminal_title" -- bash -c "./customC4; exec bash"
    sleep 1  # Add a delay to ensure the server is up before the client tries to connect

    cd pClient
    # Start the client
    ./run.sh -c 4 -p 0 -r myagent -h 127.0.0.1 -f fname$i 
    
    # Navigate to simulation directory
    cd ../simulator

    # Get the scores and add them to total scores
    mapping_score=$(awk -f mapping_score.awk planning.out ../pClient/fname$i.map | awk -F':' '{print $2}')
    total_mapping_score=$(echo $total_mapping_score + $mapping_score | bc)

    planning_score=$(awk -f planning_score.awk planning.out ../pClient/fname$i.path | grep 'Planning score:' | awk '{print $NF}')
    total_planning_score=$(echo $total_planning_score + $planning_score | bc)

    # Navigate back to pClient for the next iteration
    cd ../pClient


done

# Calculate averages
average_mapping_score=$(echo "scale=2; $total_mapping_score / 10" | bc)
average_planning_score=$(echo "scale=2; $total_planning_score / 10" | bc)

# Print averages
echo "Average Mapping Score: $average_mapping_score"
echo "Average Planning Score: $average_planning_score"
