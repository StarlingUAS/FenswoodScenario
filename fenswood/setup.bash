# Add fenswood.world to the Gazebo resource path
export GAZEBO_RESOURCE_PATH=/ros.env.d/fenswood/worlds:${GAZEBO_RESOURCE_PATH}

# Add the 'grass_box' model to the Gazebo model path
export GAZEBO_MODEL_PATH=/ros.env.d/fenswood/models:${GAZEBO_MODEL_PATH}

# Make the default media files work
cp -r /usr/share/gazebo-11/media /root/gzweb/http/client/assets/
(cd /root/gzweb/http/client/assets/media/materials/textures \
    && for f in *jpg; do convert $f ${f%.*}.png; done)