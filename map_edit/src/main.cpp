/**
******************************************************************************
*
* @file       main.cpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2026.4.10
* @version    V1.0.0
* @brief      map_edit 程序入口，启动主窗口。
*
*****************************************************************************/

#include "MainWindow.hpp"

#include <QApplication>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("map_edit"));
    QApplication::setOrganizationName(QStringLiteral("ros_map_edit"));

    int rc = 0;
    {
        MainWindow w;
        w.show();
        rc = app.exec();
    }
    rclcpp::shutdown();
    return rc;
}
