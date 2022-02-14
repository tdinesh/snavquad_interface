chmod a+x /data/home_linaro/ws_noetic/install/share/snavquad_interface/scripts/tmux_tag.sh
docker stop voxl_noetic_docker
sleep 2s
docker start voxl_noetic_docker
docker exec -it voxl_noetic_docker bash -c /root/home_linaro/ws_noetic/install/share/snavquad_interface/scripts/tmux_tag.sh