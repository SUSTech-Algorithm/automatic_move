# automatic_move

运行前请先确保已经下载rcline

```bash
  pip install rcline
```
rcline的最新教程在https://github.com/waliwuao/rcline.git

## gazebo 
好像突然可以将solidworks的模型导入了。替换模型的时候记得让solidworks导出.stl文件

感觉唯一需要做的就是：需要将模型路径手动导入$GAZEBO_MODEL_PATH **当然，前提是你把gazebo安装了**
``` .bashrc
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/automatic_move/src/world_models/models
```
