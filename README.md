# ICP 点云配准算法
## 一、环境
PCL 1.8.1

## 二、效果
可以实现键盘控制每次ICP迭代，按空格执行下一次迭代，按q退出。

![result](./result/result.png)

ICP算法对点云密度差不不大时配准效果较好(和初始点选择有关)，而当点云密度差距较大时不能很好的配准，容易卡在一个鞍点或局部极小点上。

## 三、参考资料

[1] [csdn-ICP原理与PCL代码实现](https://blog.csdn.net/qq_29462849/article/details/85080518)

 







