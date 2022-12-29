#pragma once
#include <Dense>
#include <Core>
#include <QtWidgets/QMainWindow>
#include "ui_possionedit.h"
#include "PointAndLine.h"
#include <SparseCholesky>
#include <Sparse>
#include <SparseLU>
#include "modeleditor.h"
#include "pictureeditor.h"
#include <vector>
using namespace Eigen;
using namespace std;
class possionedit : public QMainWindow
{
    Q_OBJECT
protected://�¼�����
    void paintEvent(QPaintEvent* ev);
    void mouseMoveEvent(QMouseEvent* ev);
    void mousePressEvent(QMouseEvent* ev);
    void mouseReleaseEvent(QMouseEvent* ev);
public:
    possionedit(QWidget *parent = Q_NULLPTR);
    int type;  //����Ƿ��ڻ�ͼ�����Ƶ���ʲô
    pictureeditor p;  //�Ծ�����в���(ԭʼ����
    modeleditor e;   //���������в�����Ŀ������
public slots://��������
    void OnDraw();
    void Openpic_1();
    void Openpic_2();
    void undo();
    void possion();
private:

    typedef struct edge {  //ɨ�����㷨���
        double current;
        double delta_x;
        int ymax;
    }edge;
    vector<vector<edge>> NET;
    vector<edge> AET; //��߱�
    edge eg;
    short mapP[405][405];  //��Щ�Ƕ��㣬��Щ�ᱻ����

    QVector<QVector<short>> regionBool;  //����ı߽�ֵΪ2���ڲ�ֵΪ1������Ϊ0
    QVector<QVector<long>> region;  //���Ŵ��ϵ��£������ҶԵ���б�ţ�������ٷ���
    SimplicialLDLT<SparseMatrix<double>> solver;  //�������

    Ui::possioneditClass ui;

    void possion_pre();
    void scan();
    QPoint origin_pos;  //һ��ʼ�϶�ʱ����������ڵ����λ�� �� ������Ծ��벻��
    QPoint left_up;  //����߿����Ͻ�
    QPoint right_down;  //����߿����½�

    bool pictureopened1 = 0;  //ͼƬ�Ƿ񱻴�
    bool pictureopened2 = 0;

    bool OnMoving = 0;  //�Ƿ�����ק
    long num = 0;
    short mapF[405][405];  //��Ŀ����������������������ڴ�1���������0��
    QVector<PointAndLine*> graphics;//��������
    QPen pen_pub;//���ñ�
    QImage myImage1;//��ǰͼ��
    QImage myImage1_origin;//ԭʼͼ��
    QImage image_part2;  //��Ŀ��ͼ���н�ȡ��һ����
    QImage image_part1;  //��ԭʼͼ���н�ȡ��һ����
    QImage myImage2;//��ǰͼ��
    QImage myImage2_origin;//ԭʼͼ��
    QPixmap map;
};
