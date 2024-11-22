#!/bin/bash

# Loop from 1 to 6
for i in {1..6}
do
    # Create directory
    mkdir -p $i
    
    # Create inp.txt and out.txt in each directory
    touch $i/inp.txt
    touch $i/out.txt
    
    echo "Created folder $i with inp.txt and out.txt"
done

echo "All folders and files have been created!"