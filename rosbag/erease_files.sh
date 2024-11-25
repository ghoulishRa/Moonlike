#!/bin/bash

sudo find timestamp -type f \( -name "*.jpg" -o -name "*.pcd" -o -name "*.txt" \) -exec rm -f {} +
