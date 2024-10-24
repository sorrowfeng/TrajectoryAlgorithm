# C++实现获取小球在任意路径上的圆心滚动路径
# 效果
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/eadbb3d97b684a4397ce7f9ec2e72fca.gif)


# 使用示例
```cpp
    // 准备点集
    std::vector<tra::PointF> points;
    for () { // 添加点
        double x;  
        double y;
        points.emplace_back(tra::PointF(x, y));
    }
    // 小球半径
    double radius = 50;

    // 调用接口，返回点集
    std::vector<tra::PointF> result = tra::GetTrajectoryPathWithRadius(points, radius);
	
	// 得到小球滚动路径的点集
	result;
```
# 源码

有帮助请点个star哦

> https://github.com/sorrowfeng/TrajectoryAlgorithm