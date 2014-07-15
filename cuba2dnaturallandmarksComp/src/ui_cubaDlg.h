/********************************************************************************
** Form generated from reading UI file 'cubaDlg.ui'
**
** Created: Sun Apr 3 13:52:16 2011
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CUBADLG_H
#define UI_CUBADLG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_cubaDlg
{
public:
    QGroupBox *groupBox;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *pbStop;
    QPushButton *pbResetOdometer;
    QPushButton *pushButton;
    QGroupBox *groupBox_2;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLCDNumber *lcdZ;
    QLCDNumber *lcdX;
    QLCDNumber *lcdAlfa;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QGroupBox *groupBox_3;
    QFrame *frameLaser;
    QCheckBox *check_first;
    QCheckBox *check_third;
    QCheckBox *check_second;
    QDoubleSpinBox *kminBox;
    QDoubleSpinBox *kmaxBox;
    QLabel *label_15;
    QLabel *label_16;
    QDoubleSpinBox *CornerTh;
    QLabel *label_17;
    QGroupBox *groupBox_5;
    QFrame *frameWorld;

    void setupUi(QWidget *cubaDlg)
    {
        if (cubaDlg->objectName().isEmpty())
            cubaDlg->setObjectName(QString::fromUtf8("cubaDlg"));
        cubaDlg->resize(930, 728);
        groupBox = new QGroupBox(cubaDlg);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 0, 331, 191));
        verticalLayoutWidget = new QWidget(groupBox);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 80, 151, 101));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        pbStop = new QPushButton(verticalLayoutWidget);
        pbStop->setObjectName(QString::fromUtf8("pbStop"));

        verticalLayout->addWidget(pbStop);

        pbResetOdometer = new QPushButton(verticalLayoutWidget);
        pbResetOdometer->setObjectName(QString::fromUtf8("pbResetOdometer"));

        verticalLayout->addWidget(pbResetOdometer);

        pushButton = new QPushButton(verticalLayoutWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout->addWidget(pushButton);

        groupBox_2 = new QGroupBox(groupBox);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(170, 50, 151, 131));
        gridLayoutWidget = new QWidget(groupBox_2);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(6, 18, 140, 110));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        lcdZ = new QLCDNumber(gridLayoutWidget);
        lcdZ->setObjectName(QString::fromUtf8("lcdZ"));
        QFont font;
        font.setPointSize(8);
        lcdZ->setFont(font);
        lcdZ->setFrameShadow(QFrame::Plain);
        lcdZ->setLineWidth(0);
        lcdZ->setSegmentStyle(QLCDNumber::Flat);

        gridLayout->addWidget(lcdZ, 2, 1, 1, 1);

        lcdX = new QLCDNumber(gridLayoutWidget);
        lcdX->setObjectName(QString::fromUtf8("lcdX"));
        lcdX->setFont(font);
        lcdX->setFrameShadow(QFrame::Plain);
        lcdX->setLineWidth(0);
        lcdX->setSegmentStyle(QLCDNumber::Flat);

        gridLayout->addWidget(lcdX, 0, 1, 1, 1);

        lcdAlfa = new QLCDNumber(gridLayoutWidget);
        lcdAlfa->setObjectName(QString::fromUtf8("lcdAlfa"));
        lcdAlfa->setFont(font);
        lcdAlfa->setFrameShadow(QFrame::Plain);
        lcdAlfa->setLineWidth(0);
        lcdAlfa->setSegmentStyle(QLCDNumber::Flat);

        gridLayout->addWidget(lcdAlfa, 3, 1, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 0, 0, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_3, 3, 0, 1, 1);

        groupBox_3 = new QGroupBox(cubaDlg);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(340, 0, 581, 191));
        frameLaser = new QFrame(groupBox_3);
        frameLaser->setObjectName(QString::fromUtf8("frameLaser"));
        frameLaser->setGeometry(QRect(270, 30, 300, 150));
        frameLaser->setFrameShape(QFrame::StyledPanel);
        frameLaser->setFrameShadow(QFrame::Raised);
        check_first = new QCheckBox(groupBox_3);
        check_first->setObjectName(QString::fromUtf8("check_first"));
        check_first->setGeometry(QRect(10, 20, 231, 20));
        check_third = new QCheckBox(groupBox_3);
        check_third->setObjectName(QString::fromUtf8("check_third"));
        check_third->setGeometry(QRect(10, 80, 191, 20));
        check_second = new QCheckBox(groupBox_3);
        check_second->setObjectName(QString::fromUtf8("check_second"));
        check_second->setGeometry(QRect(10, 50, 211, 20));
        kminBox = new QDoubleSpinBox(groupBox_3);
        kminBox->setObjectName(QString::fromUtf8("kminBox"));
        kminBox->setGeometry(QRect(60, 140, 51, 25));
        kminBox->setDecimals(0);
        kminBox->setMaximum(6001);
        kminBox->setValue(5);
        kmaxBox = new QDoubleSpinBox(groupBox_3);
        kmaxBox->setObjectName(QString::fromUtf8("kmaxBox"));
        kmaxBox->setGeometry(QRect(60, 110, 51, 25));
        kmaxBox->setDecimals(0);
        kmaxBox->setMaximum(10000);
        kmaxBox->setValue(10);
        label_15 = new QLabel(groupBox_3);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(0, 120, 71, 16));
        label_16 = new QLabel(groupBox_3);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(-1, 149, 71, 16));
        CornerTh = new QDoubleSpinBox(groupBox_3);
        CornerTh->setObjectName(QString::fromUtf8("CornerTh"));
        CornerTh->setGeometry(QRect(200, 110, 61, 25));
        CornerTh->setDecimals(2);
        CornerTh->setMaximum(10000);
        CornerTh->setSingleStep(0.05);
        CornerTh->setValue(0.2);
        label_17 = new QLabel(groupBox_3);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(110, 117, 111, 16));
        groupBox_5 = new QGroupBox(cubaDlg);
        groupBox_5->setObjectName(QString::fromUtf8("groupBox_5"));
        groupBox_5->setGeometry(QRect(10, 190, 911, 521));
        frameWorld = new QFrame(groupBox_5);
        frameWorld->setObjectName(QString::fromUtf8("frameWorld"));
        frameWorld->setGeometry(QRect(10, 30, 891, 481));
        frameWorld->setFrameShape(QFrame::StyledPanel);
        frameWorld->setFrameShadow(QFrame::Raised);

        retranslateUi(cubaDlg);

        QMetaObject::connectSlotsByName(cubaDlg);
    } // setupUi

    void retranslateUi(QWidget *cubaDlg)
    {
        cubaDlg->setWindowTitle(QApplication::translate("cubaDlg", "Cuba Features", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("cubaDlg", "cuba2dnaturallandmarksComp @Robolab2010", 0, QApplication::UnicodeUTF8));
        pbStop->setText(QApplication::translate("cubaDlg", "Stop", 0, QApplication::UnicodeUTF8));
        pbResetOdometer->setText(QApplication::translate("cubaDlg", "Reset Odometer", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("cubaDlg", "Save Image", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("cubaDlg", "Robot Pose", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("cubaDlg", "X ", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("cubaDlg", "Z ", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("cubaDlg", "Alfa", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("cubaDlg", "Curvature", 0, QApplication::UnicodeUTF8));
        check_first->setText(QApplication::translate("cubaDlg", "First segmentation stage ", 0, QApplication::UnicodeUTF8));
        check_third->setText(QApplication::translate("cubaDlg", "Global curvature functions", 0, QApplication::UnicodeUTF8));
        check_second->setText(QApplication::translate("cubaDlg", "Second segmentation stage", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("cubaDlg", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'DejaVu Sans'; font-weight:600;\">K</span><span style=\" font-family:'DejaVu Sans'; font-size:8pt; font-weight:600;\">max</span></p>\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'DejaVu Sans'; font-size:8pt; font-weight:600;\"></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("cubaDlg", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'DejaVu Sans'; font-weight:600;\">K</span><span style=\" font-family:'DejaVu Sans'; font-size:8pt; font-weight:600;\">min</span></p>\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'DejaVu Sans'; font-size:8pt; font-weight:600;\"></p></body></html>", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("cubaDlg", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt; font-weight:600;\">CornerTh </span></p>\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:8pt; font-weight:600;\"></p></body></html>", 0, QApplication::UnicodeUTF8));
        groupBox_5->setTitle(QApplication::translate("cubaDlg", "Robot view", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class cubaDlg: public Ui_cubaDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CUBADLG_H
