#include "possionedit.h"
#include "QFileDialog"
#include <QMessageBox>
#include <QImage>
#include <QPainter>
#include <QMouseEvent>
#include <QDesktopWidget>
#include <QApplication>
#include"PointAndLine.h"
#include"modeleditor.h"
#include"pictureeditor.h"
using namespace Eigen;
#define TYPE int
int cmp(const void* p1, const void* p2) {//�ȽϺ���
    if (*(TYPE*)p1 > *(TYPE*)p2)return 1;
    else if (*(TYPE*)p1 == *(TYPE*)p2)return 0;
    else return -1;
}
possionedit::possionedit(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    ui.label_1->setStyleSheet("QLabel{border:2px solid rgb(255, 0, 0);}");
    ui.label_2->setStyleSheet("QLabel{border:2px solid rgb(255, 0, 0);}");
    connect(ui.actiondraw, SIGNAL(triggered()), this, SLOT(OnDraw()));
    connect(ui.pushButton_1, SIGNAL(clicked()), this, SLOT(Openpic_1()));
    connect(ui.pushButton_2, SIGNAL(clicked()), this, SLOT(Openpic_2()));
    connect(ui.actionundo, SIGNAL(triggered()), this, SLOT(undo()));
    memset(mapF, 0, sizeof(mapF));  //��ʼ��
    memset(mapP, 0, sizeof(mapP));
    for (register int i = 0; i < 405; ++i) {
        vector<edge> n;
        NET.push_back(n);
    }
}
void possionedit::Openpic_1()//�������
{
    QString fileName;
    QFileDialog* fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(tr("Open File"));
    fileDialog->setFileMode(QFileDialog::AnyFile);
    fileDialog->setViewMode(QFileDialog::Detail);
    fileDialog->setGeometry(600, 600, 1000, 500);
    fileDialog->setDirectory(".\\resouses\\");  //���·��
    fileDialog->setNameFilter("Image Files(*.jpg)");
    fileDialog->show();//���ļ��Ի���
    
    if (fileDialog->exec())
    {
        fileName = fileDialog->selectedFiles()[0];
    }
    if (fileName.isEmpty()) {//���û�гɹ���
        QMessageBox::warning(this, "Warning!", "Fail to open!");
        return;
    }
    myImage1.load(fileName);
    pictureopened1 = 1;
    map = QPixmap::fromImage(myImage1);

    ui.label_1->setAlignment(Qt::AlignCenter);
    ui.label_1->setPixmap(QPixmap::fromImage(myImage1).scaled(ui.label_1->size()));
    myImage1 = myImage1.scaled(ui.label_1->size());//���ݻ���������С
    myImage1_origin = myImage1;//����һ��ԭʼͼƬ

    undo();  //���֮ǰ�Ĳ���

}
void possionedit::undo() {//�������
    if (graphics.size() == 0)return;
    memset(mapF, 0, sizeof(mapF));
    memset(mapP, 0, sizeof(mapP));
    myImage1 = myImage1_origin;
    myImage2 = myImage2_origin;
    regionBool.clear();
    region.clear();
    NET.clear();
    for (register int i = 0; i < 405; ++i) {
        vector<edge> n;
        NET.push_back(n);
    }
    AET.clear();
    while (graphics.size() != 0) {
        PointAndLine* u = graphics.last();
        graphics.pop_back();
        free(u);
    } 
    ui.label_1->setPixmap(QPixmap::fromImage(myImage1).scaled(ui.label_1->size())); 
    ui.label_2->setPixmap(QPixmap::fromImage(myImage2).scaled(ui.label_2->size()));
    update();
    
}
void possionedit::Openpic_2()
{   //�������
    QString fileName;
    QFileDialog* fileDialog = new QFileDialog(this);
    fileDialog->setWindowTitle(tr("Open File"));
    fileDialog->setFileMode(QFileDialog::AnyFile);
    fileDialog->setViewMode(QFileDialog::Detail);
    fileDialog->setGeometry(600, 600, 1000, 500);
    fileDialog->setDirectory(".\\resouses\\");
    fileDialog->setNameFilter("Image Files(*.jpg)");
    fileDialog->show();//���ļ��Ի���

    if (fileDialog->exec())
    {
        fileName = fileDialog->selectedFiles()[0];
    }
    if (fileName.isEmpty()) {//���û�гɹ���
        QMessageBox::warning(this, "Warning!", "Fail to open!");
        return;
    }
    myImage2.load(fileName);
    pictureopened2 = 1;
    map = QPixmap::fromImage(myImage2);

    ui.label_2->setAlignment(Qt::AlignCenter);
    ui.label_2->setPixmap(QPixmap::fromImage(myImage2).scaled(ui.label_2->size()));
    myImage2 = myImage2.scaled(ui.label_2->size());//���ݻ���������С
    myImage2_origin = myImage2;//����һ��ԭʼͼƬ
    undo();  //���֮ǰ�Ĳ���
}
void possionedit::OnDraw() {//����״̬
    if(graphics.isEmpty())
    type = 1;
    else {
        QMessageBox box(QMessageBox::Warning, "����", "�Ѿ�������");
        box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        box.exec();//����
    }
}
void possionedit::paintEvent(QPaintEvent*)
{
    if (!pictureopened1|| !pictureopened2 || graphics.isEmpty())return;
    //ͼ��û�д򿪣�ֹͣ�滭   //ע����һ��ͼ����û�д�
    map = QPixmap::fromImage(myImage1).scaled(ui.label_1->size());
    QPainter p1(&map);
    PointAndLine& line = *graphics.at(0);
    for (int j = 0; j < line.path.size() - 1; j++) {
        p1.setPen(line.pen);
        p1.drawLine(line.path.at(j), line.path.at(j + 1));
    }
    ui.label_1->setPixmap(map); //����ͼ��
    p1.end();
    return;//����ͼ��
}
void possionedit::mousePressEvent(QMouseEvent* ev)
{
    if (graphics.size() >= 2)return;  //�Ѿ������䶨��
    if (!pictureopened1 || !pictureopened1) {
        QMessageBox box(QMessageBox::Warning, "����", "ͼ��û�д�");
        box.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        box.exec();//����
    }
    if (ui.label_1->geometry().contains(ui.label_1->mapFromParent(ui.centralWidget->mapFromParent(ev->pos()))) && pictureopened1&& pictureopened2)
    {
        QPoint s = ui.label_1->mapFromParent(ui.centralWidget->mapFromParent(ev->pos())); //�ڻ����е�λ��
        if (mapF[s.x()][s.y()] != 0) {
            OnMoving = 1;  //��Ŀ�����������ק
            origin_pos = s;
            PointAndLine* r = new PointAndLine; //��ʼ��
            r->type = 2;
            pen_pub.setColor(QColor(0, 255, 0));
            pen_pub.setWidth(1);
            r->pen = pen_pub;
            r->path.append(s);   //��image2�ж�Ӧ���ⷽ��
            r->path.append(s);
            r->path.append(s);
            r->path.append(s);
            r->path.append(s);
            graphics.append(r);
        }

        if (type == 0)return;//ûѡ��
        if (type == 1) {
            PointAndLine* r = new PointAndLine; //��ʼ��
            r->type = 1;
            pen_pub.setColor(QColor(0, 0, 255));
            pen_pub.setWidth(3);
            r->pen = pen_pub;
            r->path.append(s);
            graphics.append(r);
        }
    }
    update();
}
void possionedit::mouseMoveEvent(QMouseEvent* ev)//���Ʋ�����Ͳ�����
{
    if (pictureopened1 && pictureopened2 && !graphics.isEmpty()) {
        if (OnMoving && ui.label_2->geometry().contains(this->mapFromGlobal(QCursor::pos()))) {//������קĿ������
            PointAndLine* lastLine = graphics.last();
            int type_1 = (*graphics.last()).type;
            if (type_1 == 2) {
                lastLine->draw(ui.label_2->mapFromParent(ui.centralWidget->mapFromParent(ev->pos())) - origin_pos + left_up, ui.label_2->mapFromParent(ui.centralWidget->mapFromParent(ev->pos())) - origin_pos + right_down);
            }
            possion();  //ʵʱ����
            update();
        }
        if (ui.label_1->geometry().contains(ui.label_1->mapFromParent(ui.centralWidget->mapFromParent(ev->pos()))) && !OnMoving)//���ڻ���Ŀ������
        {
            if (type == 0)return;
            PointAndLine* lastLine = graphics.last();
            int type_1 = (*graphics.last()).type;
            if (type_1 == 1) {//���ɻ�ͼ
                QPoint s = lastLine->path.last();
                QPoint t = ui.label_1->mapFromParent(ui.centralWidget->mapFromParent(ev->pos()));
                QPoint v;
                if (abs(s.x() - t.x()) + abs(s.y() - t.y()) > 6) {//�����ظ�����
                    lastLine->draw(t);
                    if (s.y() != t.y()) { //������ɨ����ƽ��
                        if (lastLine->path.size() > 1) {//���¶���
                            QPoint r = lastLine->path.at(lastLine->path.size() - 2);
                            int flag = (s.y() - r.y()) * (s.y() - t.y());//�ж��ǲ�����ͬһ��
                            if (flag > 0) {
                                mapP[s.x()][s.y()] = 1;   //ͬһ�� �� ��ֵ
                            }
                            else if (flag < 0) {
                                mapP[s.x()][s.y()] = 0;    //��� �� ʵֵ
                            }
                            else {
                                mapP[s.x()][s.y()] = 0;   //ƽ�� �� ʵֵ
                            }
                        }
                        if (s.y() > t.y()) {
                            v = s;
                            s = t;
                            t = v;
                        }
                        eg.current = s.x();
                        eg.delta_x = (double)(t.x() - s.x()) / (double)(t.y() - s.y());
                        eg.ymax = t.y();
                        NET.at(s.y()).push_back(eg); //���NET
                    }
                    else {//�Զ�����и���
                        mapP[s.x()][s.y()] = 0; //ƽ��ʱ����¼�ߣ������Ϊʵֵ
                    }

                }
            }
            update();
        }
    }
}

void possionedit::scan() {//ɨ�����㷨
    bool isempty = 1;
    short isinterior = -1;
    int x = 0;
    int y = 0;
    int k = 0;
    int linecut[405];
    for (register int i = 0; i < 405; ++i) {//ɨ��ÿһ��
        isinterior = -1;
        k = 0;
        memset(linecut, 0, sizeof(linecut));
        for (register int j = 0; j < NET.at(i).size(); ++j) {
            AET.push_back(NET.at(i).at(j));   //��NET�д�i��������Щ��ӵ�AET��
        }
        for (register int j = 0; j < AET.size(); ++j) {
            if (AET.at(j).ymax == i) {//����ߵ�����ֵ
                AET.erase(AET.begin() + j); //ɾ��
                --j;
            }
            else {
                int b = (int)(AET.at(j).current + 0.5);//��������
                if (mapP[b][i] == 0) {
                    linecut[k] = b;
                    k++;
                }
                AET.at(j).current += AET.at(j).delta_x;  //ÿһ���ཻ�ı߼�delta
            }
        }
        qsort(linecut, k, sizeof(int), cmp);  //�Խ����������
        int u = linecut[0];
        for (register int j = 0; j < k-1; ++j) {//���
            isinterior = -isinterior;
            while (u != linecut[j + 1]) {
                if(isinterior == 1)mapF[u][i] = 1;
                u++;
            }
            
        }
    }
    //������ȷ�ϱ߿�
    while (isempty) {
        for (register int j = 0; j < 405; ++j) {
            if (mapF[x][j] != 0)isempty = 0;
        }
        x++;
    }
    left_up.setX(x-1);
    isempty = 1;
    while (isempty) {
        for (register int j = 0; j < 405; ++j) {
            if (mapF[j][y] != 0)isempty = 0;
        }
        y++;
    }
    left_up.setY(y - 1);
    isempty = 1;
    x = 404;
    while (isempty) {
        for (register int j = 0; j < 405; ++j) {
            if (mapF[x][j] != 0)isempty = 0;
        }
        x--;
    }
    right_down.setX(x + 1);
    isempty = 1;
    y = 404;
    while (isempty) {
        for (register int j = 0; j < 405; ++j) {
            if (mapF[j][y] != 0)isempty = 0;
        }
        y--;
    }
    right_down.setY(y + 1);

    for (register int i = 0; i < 405; ++i) { //���Ʊ߽�
        for (register int j = 0; j < 405; ++j) {
            if (mapF[i][j] == 1) {
                if ((i > 0 && mapF[i - 1][j] == 0) || (i<404&& mapF[i+1][j]==0) || (j > 0 && mapF[i][j-1] == 0) || (j < 404 && mapF[i ][j+1] == 0)) {
                    mapF[i][j] = 2;                   
                }
            }
        }
    }

}
void possionedit::mouseReleaseEvent(QMouseEvent* ev) {
    if (graphics.size() == 1 && type == 1) {
        PointAndLine* lastLine = graphics.last();
        QPoint s = lastLine->path.last();
        QPoint t = lastLine->path.at(0);
        lastLine->draw(t);
        QPoint v;
        if (s.y() != t.y()) { //������ɨ����ƽ��
            if (lastLine->path.size() > 1) {//���¶���
                QPoint r = lastLine->path.at(lastLine->path.size() - 2);
                int flag = (s.y() - r.y()) * (s.y() - t.y());//�ж��ǲ�����ͬһ��
                if (flag > 0) {
                    mapP[s.x()][s.y()] = 1;   //ͬһ�� �� ��ֵ
                }
                else if (flag < 0) {
                    mapP[s.x()][s.y()] = 0;    //��� �� ʵֵ
                }
                else {
                    mapP[s.x()][s.y()] = 0;   //ƽ�� �� ʵֵ
                }
            }
            if (s.y() > t.y()) {
                v = s;
                s = t;
                t = v;
            }
            eg.current = s.x();
            eg.delta_x = (double)(t.x() - s.x()) / (double)(t.y() - s.y());
            eg.ymax = t.y();
            NET.at(s.y()).push_back(eg); //���NET
        }
        else {//�Զ�����и���
            mapP[s.x()][s.y()] = 0; //ƽ��ʱ����¼�ߣ������Ϊʵֵ
            //if (lastLine->path.size() > 1)QPoint r = lastLine->path.at(lastLine->path.size() - 2);
        }
       

        s = lastLine->path.last();  //�����������ж�
        t = lastLine->path.at(1);
        QPoint r = lastLine->path.at(lastLine->path.size() - 2);
        int flag = (s.y() - r.y()) * (s.y() - t.y());//�ж��ǲ�����ͬһ��
        if (flag > 0) {
            mapP[s.x()][s.y()] = 1;   //ͬһ�� �� ��ֵ
        }
        else if (flag < 0) {
            mapP[s.x()][s.y()] = 0;    //��� �� ʵֵ
        }
        else {
            mapP[s.x()][s.y()] = 0;   //ƽ�� �� ʵֵ
        }

        scan();
        possion_pre();
        graphics.at(0)->pen.setColor(qRgb(0, 255, 0));
    }
    type = 0;
    OnMoving = 0;
    
}
void possionedit::possion_pre() {

    QVector<short> ra, ra1 ;
    QVector<long>  rb, rb1;
    long k = 0;
    //��ʼ��region �� regionBool
    for (register int i = 0; i <= right_down.x()-left_up.x(); ++i) {
        ra1.append(0);
        rb1.append(0);
    }
    for (register int i = 0; i <= right_down.y() - left_up.y(); ++i) {
        ra = ra1;
        rb = rb1;
        for (register int j = 0; j <= right_down.x() - left_up.x(); ++j) {
            if (mapF[left_up.x() + j][left_up.y() + i] != 0) {
                ra.replace(j, mapF[left_up.x() + j][left_up.y() + i]);
                rb.replace(j, k);
                k++;
            }
           
        }
        regionBool.append(ra);
        region.append(rb);
    }
    image_part1 = myImage1_origin.copy(left_up.x(), left_up.y(), right_down.x() - left_up.x()+1, right_down.y() - left_up.y()+1);
    e.setregionBool(regionBool);
    e.setregion(region);
    e.setPosnum(k);
    e.initial();
    num = k;
    p.setimage1(image_part1);
    p.setMatrix(e.getmatrix());
    p.setregionBool(regionBool);
    p.setnum(num);
    p.initial();
}
void possionedit::possion() {
    image_part2 = myImage2_origin.copy((*(graphics.last())).path.at(0).x(), (*(graphics.last())).path.at(0).y(), abs((*(graphics.last())).path.at(2).x() - (*(graphics.last())).path.at(0).x())+1, abs((*(graphics.last())).path.at(2).y() - (*(graphics.last())).path.at(0).y())+1);
    p.setimage2(image_part2);
    p.analysis(); //����
    myImage2 = myImage2_origin;  //��ȡ���������ͼ��
    QPainter imagepainter(&myImage2);  //�½�����
    imagepainter.setCompositionMode(QPainter::CompositionMode_SourceOver);   //�����ص�Ч��
    imagepainter.drawImage((*(graphics.last())).path.at(0).x(), (*(graphics.last())).path.at(0).y(), p.getimage());
    imagepainter.end();
    ui.label_2->setPixmap(QPixmap::fromImage(myImage2).scaled(ui.label_2->size()));//����

}

