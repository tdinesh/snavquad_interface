chmod a+x /data/home_linaro/ws_melodic/install/share/snavquad_interface/scripts/tmux_tag.sh
docker stop voxl_melodic_docker
sleep 2s
docker start voxl_melodic_docker 
docker exec -it voxl_melodic_docker bash -c /root/home_linaro/ws_melodic/install/share/snavquad_interface/scripts/tmux_tag.sh
