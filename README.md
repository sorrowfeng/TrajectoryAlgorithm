# C++ implementation to obtain the rolling path of the center of a ball on any path
# Effect
![在这里插入图片描述](https://i-blog.csdnimg.cn/direct/eadbb3d97b684a4397ce7f9ec2e72fca.gif)


# Usage example
```cpp
    // Prepare a collection of points
    std::vector<tra::PointF> points;
    for () { 
        double x;  
        double y;
        points.emplace_back(tra::PointF(x, y));
    }
    // Ball radius
    double radius = 50;

    // Call the interface and return a set of points
    std::vector<tra::PointF> result = tra::GetTrajectoryPathWithRadius(points, radius);
	
	// Obtain the point set of the rolling path of the ball
	result;
```
