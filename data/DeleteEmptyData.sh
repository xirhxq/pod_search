#!/bin/bash

for file in *.csv; do
    [ -e "$file" ] || continue

    line_count=$(wc -l < "$file")

    if [ "$line_count" -le 1 ]; then
        echo "Deleting: $file"
        rm "$file"
    fi

done
