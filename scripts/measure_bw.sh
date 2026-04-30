#!/bin/bash

export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}

source /opt/ros/noetic/setup.bash
source ~/ws_livox/devel/setup.bash

TMPDIR=$(mktemp -d /tmp/bw_XXXXXX)
rm -f /tmp/bw_latest
ln -sf "$TMPDIR" /tmp/bw_latest

TOPICS=$(rostopic list 2>/dev/null)

if [ -z "$TOPICS" ]; then
    echo "ERROR: No ROS topics found."
    exit 1
fi

# Read into array
readarray -t TOPIC_ARRAY <<< "$TOPICS"
N=${#TOPIC_ARRAY[@]}

BATCH_SIZE=8
WINDOW=5
TIMEOUT=$((WINDOW + 8))

echo "Measuring bandwidth for $N topics (window=${WINDOW}s, batch=${BATCH_SIZE})..."
echo "Temp dir: $TMPDIR"
echo ""

total_batches=$(( (N + BATCH_SIZE - 1) / BATCH_SIZE ))
batch_num=0

for ((i=0; i<N; i+=BATCH_SIZE)); do
    batch_num=$((batch_num + 1))
    batch_end=$((i + BATCH_SIZE))
    if [ $batch_end -gt $N ]; then batch_end=$N; fi

    echo -n "  Batch $batch_num/$total_batches (topics $((i+1))-$batch_end): "

    # Launch this batch in parallel
    for ((j=i; j<batch_end; j++)); do
        t="${TOPIC_ARRAY[$j]}"
        fname=$(echo "$t" | tr '/' '_')
        (
            source /opt/ros/noetic/setup.bash
            source ~/ws_livox/devel/setup.bash
            timeout $TIMEOUT rostopic bw "$t" -w $WINDOW 2>/dev/null
        ) > "$TMPDIR/$fname" 2>/dev/null &
    done

    wait
    echo "done"
done

echo ""
printf "%-60s %15s\n" "Topic" "Bandwidth"
echo "-----------------------------------------------------------------------------"

has_data=0
no_msgs=0
no_data=0

for t in "${TOPIC_ARRAY[@]}"; do
    fname=$(echo "$t" | tr '/' '_')
    f="$TMPDIR/$fname"

    if [ ! -s "$f" ]; then
        printf "%-60s %15s\n" "$t" "---"
        no_data=$((no_data + 1))
        continue
    fi

    # Check for at least one "average:" line
    if ! grep -q 'average:' "$f" 2>/dev/null; then
        printf "%-60s %15s\n" "$t" "0 (no msgs)"
        no_msgs=$((no_msgs + 1))
        continue
    fi

    # Extract last average bandwidth
    bw=$(grep 'average:' "$f" | tail -1 | grep -oP 'average:\s*\K[\d.]+\s*[KMGT]?B?/s' | tr -d ' ')
    if [ -z "$bw" ]; then
        bw=$(grep 'average:' "$f" | tail -1 | awk -F': ' '{print $NF}' | awk '{print $1}')
    fi

    if [ -n "$bw" ]; then
        printf "%-60s %15s\n" "$t" "$bw"
        has_data=$((has_data + 1))
    else
        printf "%-60s %15s\n" "$t" "PARSE_ERR"
        echo "  [debug] $(tail -1 "$f")" >&2
        no_data=$((no_data + 1))
    fi
done

echo "-----------------------------------------------------------------------------"
echo "Active: $has_data   Idle: $no_msgs   Error: $no_data   Total: $N"
echo "Temp files: /tmp/bw_latest/"
echo "Done."
