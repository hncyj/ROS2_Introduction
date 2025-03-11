#include <QApplication>
#include <QLabel>
#include <QString>

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QLabel* label = new QLabel();
    QString msg = QString::fromStdString("Hello, Qt!");
    label->setText(msg);
    label->resize(200, 100);
    // 设置窗口位置
    label->move(100, 100);
    label->show();
    app.exec();

    return 0;
}