#!/bin/bash
source ${HOME}/autonomous-drone/catkin_ws/devel/setup.bash
nodenames="$(catkin list)"
for word in ${nodenames//- /$''}
do
    if [ $word != "monash_main" ]
    then
        roscd $word
        ln -sf ${HOME}/autonomous-drone/catkin_ws/src/monash_main/.clang-format .
        ln -sf ${HOME}/autonomous-drone/catkin_ws/src/monash_main/.clang-tidy .
    fi
    roscd $word
    ln -sf ${HOME}/autonomous-drone/catkin_ws/build/${word}/compile_commands.json .
done
