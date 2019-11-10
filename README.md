# disperser
输入一个stl文件

<img src="https://github.com/chloro-pn/disperser/blob/master/pic/1.png" width="450" height="300">

disperser可以将该stl实体剖分成一个个堆砌起来的网格

<img src="https://github.com/chloro-pn/disperser/blob/master/pic/2.png" width="450" height="300">

#特点：
* disperser目前仅支持二进制stl文件。
* disperser对输入的stl实体进行拓扑检查，如果该文件中有三角形面皮拼接错误则会返回错误信息。
* disperser剖分结果输出目前仅支持tecplot软件输入文件文本格式。
