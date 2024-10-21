#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPainter>
#include <QPainterPath>
#include <QDebug>

#include "TrajectoryAlgorithm.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    path.moveTo(100, 100);
    path.lineTo(200, 200);
    path.arcTo(200, 150, 300, 100, 180, -180);
    path.lineTo(2000, 800);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter p(this);

    p.setPen(Qt::black);
    p.drawPath(path);

    p.setPen(Qt::red);
    p.drawPath(pathnew);
}

void MainWindow::on_pushButton_clicked()
{
    // 准备点集
    std::vector<tra::PointF> points;
    for (double i = 0; i <= 1; i += 0.001) {
        double x = path.pointAtPercent(i).x();
        double y = path.pointAtPercent(i).y();
        points.emplace_back(tra::PointF(x, y));
    }

    // 调用接口，返回点集
    std::vector<tra::PointF> result = tra::GetTrajectoryPathWithRadius(points, 50);

    // 绘制显示
    pathnew.clear();
    for (int i = 0; i < result.size(); ++i) {
        if (i == 0) pathnew.moveTo(result.at(i).x, result.at(i).y);
        else        pathnew.lineTo(result.at(i).x, result.at(i).y);
    }

    this->update();
}

