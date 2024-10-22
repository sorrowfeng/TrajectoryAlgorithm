#ifndef TRAJECTORYALGORITHM_HPP
#define TRAJECTORYALGORITHM_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <climits>

namespace trajectory_algorithm {

// 定义浮点类型结构体
struct PointF {
    double x;
    double y;

    PointF(double x_val, double y_val) : x(x_val), y(y_val) {}
};

// 定义BinaryPoint结构体
struct BinaryPoint {
    int row;
    int col;

    BinaryPoint(int row, int col) : row(row), col(col) {}
};

// 定义BinaryMatrix结构体
struct BinaryMatrix {
    int rows;
    int cols;
    std::vector<int> data;  // 使用一维数组，优化读写效率

    BinaryMatrix(int rows, int cols) : rows(rows), cols(cols), data(rows * cols, 0) {}

    /// <summary>
    /// 设置某行列上的值
    /// </summary>
    /// <param name="row"></param>
    /// <param name="col"></param>
    /// <param name="value"></param>
    void setValue(int row, int col, int value) {
        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            data[row * cols + col] = value;
        }
    }

    /// <summary>
    /// 获取某行列上的值
    /// </summary>
    /// <param name="row"></param>
    /// <param name="col"></param>
    /// <returns></returns>
    int getValue(int row, int col) const {
        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            return data[row * cols + col];
        }
        return 0; // 或者可以选择抛出异常
    }

    /// <summary>
    /// 测试打印
    /// </summary>
    void print() const {
        for (int row = 0; row < rows; ++row) {
            for (int col = 0; col < cols; ++col) {
                auto v = data[row * cols + col] ? "1" : " ";
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }
    }

    /// <summary>
    /// 通过点与半径，设置该圆形区域所有值
    /// </summary>
    /// <param name="point">点</param>
    /// <param name="radius">半径</param>
    void setCircle(const BinaryPoint& point, int radius) {
        // for (int i = -radius; i <= radius; ++i) {
        //     for (int j = -radius; j <= radius; ++j) {
        //         if (i * i + j * j <= radius * radius) {
        //             setValue(point.row + j, point.col + i, 1);
        //         }
        //     }
        // }

        // 使用 Bresenham 算法绘制圆形，能大大优化效率
        int x = radius;
        int y = 0;
        int err = 0;

        while (x >= y) {
            setValue(point.row + x, point.col + y, 1);
            setValue(point.row + y, point.col + x, 1);
            setValue(point.row - y, point.col + x, 1);
            setValue(point.row - x, point.col + y, 1);
            setValue(point.row - x, point.col - y, 1);
            setValue(point.row - y, point.col - x, 1);
            setValue(point.row + y, point.col - x, 1);
            setValue(point.row + x, point.col - y, 1);

            if (err <= 0) {
                y += 1;
                err += 2 * y + 1;
            }
            if (err > 0) {
                x -= 1;
                err -= 2 * x + 1;
            }
        }
    }

    /// <summary>
    /// 设置点集并绘制圆形区域
    /// </summary>
    /// <param name="points">点集</param>
    /// <param name="radius">半径</param>
    void setPointsWithRadius(const std::vector<BinaryPoint>& points, int radius) {
        // 画圆
        for (const auto& point : points) {
            setCircle(point, radius);
        }
    }

    /// <summary>
    /// 遍历BinaryMatrix，按列从上到下遍历，找到第一个值为1的点，并返回这些点的数组
    /// </summary>
    /// <returns>BinaryPoint数组</returns>
    std::vector<BinaryPoint> findFirstOnesInColumns() {
        std::vector<BinaryPoint> result;

        for (int col = 0; col < cols; ++col) {
            for (int row = 0; row < rows; ++row) {
                if (getValue(row, col) == 1) {
                    result.emplace_back(row, col);
                    break; // 找到第一个值为1的点后，跳出当前列的循环
                }
            }
        }

        return result;
    }

    /// <summary>
    /// 遍历BinaryMatrix，按列从下到上遍历，找到第一个值为1的点，并返回这些点的数组
    /// </summary>
    /// <returns>BinaryPoint数组</returns>
    std::vector<BinaryPoint> findLastOnesInColumns() {
        std::vector<BinaryPoint> result;

        for (int col = 0; col < cols; ++col) {
            for (int row = rows - 1; row >= 0; --row) {
                if (getValue(row, col) == 1) {
                    result.emplace_back(row, col);
                    break; // 找到第一个值为1的点后，跳出当前列的循环
                }
            }
        }

        return result;
    }

};

/// <summary>
/// 得到半径补偿计算后的点集
/// </summary>
/// <param name="originPoints">原始点集</param>
/// <param name="radius">半径</param>
/// <returns>半径补偿后的点集</returns>
std::vector<PointF> GetTrajectoryPathWithRadius(const std::vector<PointF>& originPoints, double radius) {
    std::vector<PointF> outputPoints;

    std::vector<BinaryPoint> inputPoints;
    inputPoints.reserve(originPoints.size());

    // 放大倍数，将小数输入都放大，全部转换为int
    double factor = 10;
    double radius_e = radius * factor;

    // 获取范围，方便确定mat大小
    int min_row = std::numeric_limits<int>::max();
    int min_col = std::numeric_limits<int>::max();
    int max_row = std::numeric_limits<int>::min();
    int max_col = std::numeric_limits<int>::min();

    // 放大点，并得到范围
    for (int i = 0; i < originPoints.size(); ++i) {
        auto point = originPoints.at(i);
        double x = point.x * factor;
        double y = point.y * factor;
        int row = y;
        int col = x;
        inputPoints.emplace_back(BinaryPoint(row, col));
        if (row < min_row) min_row = row;
        if (row > max_row) max_row = row;
        if (col < min_col) min_col = col;
        if (col > max_col) max_col = col;
    }

    // 边上留出给半径写入的余量
    int rows = max_row - min_row + 4 * radius_e;
    int cols = max_col - min_col + 4 * radius_e;

    // 得到全局移动的偏移值
    min_row -= 2 * radius_e;
    min_col -= 2 * radius_e;

    // 确保范围有效
    if (min_row >= std::numeric_limits<int>::max() || min_col >= std::numeric_limits<int>::max())
        return outputPoints;
    if (max_row <= std::numeric_limits<int>::min() || max_col <= std::numeric_limits<int>::min())
        return outputPoints;

    // 移动点的相对位置，确保都大于0
    for (auto& point : inputPoints) {
        point.row -= min_row;
        point.col -= min_col;
    }

    // 创建BinaryMatrix对象
    BinaryMatrix mat(rows, cols);

    // 将点集转换为二值矩阵，并在每个点周围绘制圆形区域
    mat.setPointsWithRadius(inputPoints, radius_e);

    // 获得目标点
    std::vector<BinaryPoint> result = mat.findLastOnesInColumns();

    // 移动点的相对位置，将点移动回去，并同时缩小factor
    for (auto& point : result) {
        point.row += min_row;
        point.col += min_col;
        double row = (double)point.row / factor;
        double col = (double)point.col / factor;
        outputPoints.emplace_back(PointF(col, row));
    }

    return outputPoints;
}

}

// 定义命名空间别名
namespace tra = trajectory_algorithm;

#endif // TRAJECTORYALGORITHM_HPP
