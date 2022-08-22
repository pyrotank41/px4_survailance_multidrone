docker run -it -v "$(pwd)":/root/my_ws \
               --net=host \
              --env="DISPLAY" \
              --env="QT_X11_NO_MITSHM=1" \
              --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
              drone_img bash
