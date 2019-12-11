# disperser
输入一个stl文件

<img src="https://github.com/chloro-pn/disperser/blob/master/pic/1.png" width="450" height="300">

disperser可以将该stl实体剖分成一个个堆砌起来的网格

grid_size = 0.05

<img src="https://github.com/chloro-pn/disperser/blob/master/pic/3.png" width="450" height="300">

grid_size = 0.025

<img src="https://github.com/chloro-pn/disperser/blob/master/pic/4.png" width="450" height="300">

grid_size = 0.005

<img src="https://github.com/chloro-pn/disperser/blob/master/pic/2.png" width="450" height="300">

update:

1. 更新日志库，见pnlog。
2. 增加可以计算边界节点坐标以及法向量的模块。

#特点：
* disperser目前仅支持二进制stl文件。
* disperser对输入的stl实体进行拓扑检查，如果该文件中有三角形面片拼接错误则会返回错误信息。
* disperser剖分结果输出目前仅支持tecplot软件输入文件文本格式。
