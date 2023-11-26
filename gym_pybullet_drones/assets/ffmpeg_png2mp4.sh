#!/bin/bash

# USE
# 1. copy the script in the directory where the framse are saved: $ cp ./ffmpeg_png2mp4.sh ./video-MM-DD-YYYY_HH.MM.SS
# 2. correct the values of FRAME_RATE and RESOLUTION, if necessary (remember not to add spaces before of after the = sign)
# 3. make the script executable, if needed: $ cd ./video-MM-DD-YYYY_HH.MM.SS; $ chmod +x ./ffmpeg_png2mp4.sh
# 4. run the script to create "video.mp4" in the same directory: $ ./ffmpeg_png2mp4.sh

FRAME_RATE=24
RESOLUTION=640x480

ffmpeg -r $FRAME_RATE -f image2 -s $RESOLUTION -i frame_%d.png -vcodec libx264 -crf 25  -pix_fmt yuv420p video.mp4