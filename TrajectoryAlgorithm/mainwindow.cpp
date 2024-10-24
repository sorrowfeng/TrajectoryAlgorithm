#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QPainter>
#include <QPainterPath>
#include <QDebug>
#include <QThread>
#include <QTimer>

#include "TrajectoryAlgorithm.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    path.moveTo(100, 100);
    path.lineTo(200, 200);
    path.arcTo(200, 150, 300, 100, 180, -180);
    path.lineTo(800, 300);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter p(this);

    QTransform transform;
    transform.scale(1, -1);
    transform.translate(0, -500);
    p.setTransform(transform);

    p.setPen(Qt::black);
    p.drawPath(path);

    p.setPen(Qt::blue);
    p.drawPath(pathpos);

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
    // 半径
    double radius = 50;

    // 调用接口，返回点集
    std::vector<tra::PointF> result = tra::GetTrajectoryPathWithRadius(points, radius);




    // 绘制
    pathpos.clear();
    pathpos.addEllipse(QPointF(0, 0), 50, 50);

    QTimer* timer = new QTimer(this);

    connect(timer, &QTimer::timeout, this, [=](){
        static int index = 0;
        index++;
        if (index >= result.size()) {
            index = 0;
        }

        // 绘制显示
        pathnew.clear();
        for (int i = 0; i < index; ++i) {
            if (i == 0)
                pathnew.moveTo(result.at(i).x, result.at(i).y);
            else
                pathnew.lineTo(result.at(i).x, result.at(i).y);
        }
        auto pos_center = pathpos.boundingRect().center();
        auto pos_trans = QPointF(result.at(index).x, result.at(index).y) - pos_center;
        pathpos.translate(pos_trans);
        this->update();
    });

    timer->start(1);
}

