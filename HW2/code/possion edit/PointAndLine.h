#pragma once
#include <vector>
#include <QPainter>
#include <QMouseEvent>
class PointAndLine//���ڲ�����Ͳ����ߣ���minidraw�е���
{
public:
    int type;
    QVector<QPoint>path;
    QPen pen;
    PointAndLine() {

    }
    ~PointAndLine() {

    }
    void draw(QPoint pos) { //���ɻ���
        path.append(pos);
    }
    void draw(QPoint pos1 , QPoint pos2) {  //���Ʊ߿�
        path.replace(0 , pos1);
        QPoint p(pos1.x(), pos2.y());
        path.replace(1, p);//��Ӧ��������ĸ���
        QPoint q(pos2.x(), pos1.y());
        path.replace(3, q);
        path.replace(2, pos2);
        path.replace(4, path.at(0));
    }
};


