#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPainterPath>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    // QWidget interface
protected:
    virtual void paintEvent(QPaintEvent *event) override;

private slots:
    void on_pushButton_clicked();

private:
    QPainterPath path;
    QPainterPath pathnew;
};
#endif // MAINWINDOW_H
