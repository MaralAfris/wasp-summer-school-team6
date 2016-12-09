#!/bin/bash
ffmpeg -pattern_type glob -i 'video/*.png' -r 25 'video.mp4'
