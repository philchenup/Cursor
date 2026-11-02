/********************************************************************************
** Form generated from reading UI file 'record.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RECORD_H
#define UI_RECORD_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_Record
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QPushButton *startRecordBtn;
    QPushButton *pauseRecordBtn;
    QPushButton *roiRecordBtn;
    QPushButton *exitRecordBtn;

    void setupUi(QDialog *Record)
    {
        if (Record->objectName().isEmpty())
            Record->setObjectName(QString::fromUtf8("Record"));
        Record->resize(317, 38);
        Record->setMaximumSize(QSize(317, 16777215));
        Record->setAutoFillBackground(false);
        Record->setStyleSheet(QString::fromUtf8("font: 9pt \"\345\276\256\350\275\257\351\233\205\351\273\221\";"));
        verticalLayout = new QVBoxLayout(Record);
        verticalLayout->setSpacing(3);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(3, 3, 3, 3);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(3);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        startRecordBtn = new QPushButton(Record);
        startRecordBtn->setObjectName(QString::fromUtf8("startRecordBtn"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/res/icon/start_.svg"), QSize(), QIcon::Normal, QIcon::Off);
        startRecordBtn->setIcon(icon);

        horizontalLayout->addWidget(startRecordBtn);

        pauseRecordBtn = new QPushButton(Record);
        pauseRecordBtn->setObjectName(QString::fromUtf8("pauseRecordBtn"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/res/icon/pause.svg"), QSize(), QIcon::Normal, QIcon::Off);
        pauseRecordBtn->setIcon(icon1);

        horizontalLayout->addWidget(pauseRecordBtn);

        roiRecordBtn = new QPushButton(Record);
        roiRecordBtn->setObjectName(QString::fromUtf8("roiRecordBtn"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/res/icon/setting.svg"), QSize(), QIcon::Normal, QIcon::Off);
        roiRecordBtn->setIcon(icon2);

        horizontalLayout->addWidget(roiRecordBtn);

        exitRecordBtn = new QPushButton(Record);
        exitRecordBtn->setObjectName(QString::fromUtf8("exitRecordBtn"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/res/icon/application-exit.svg"), QSize(), QIcon::Normal, QIcon::Off);
        exitRecordBtn->setIcon(icon3);

        horizontalLayout->addWidget(exitRecordBtn);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(Record);

        QMetaObject::connectSlotsByName(Record);
    } // setupUi

    void retranslateUi(QDialog *Record)
    {
        Record->setWindowTitle(QCoreApplication::translate("Record", "Dialog", nullptr));
        startRecordBtn->setText(QCoreApplication::translate("Record", "\345\274\200\345\247\213", nullptr));
        pauseRecordBtn->setText(QCoreApplication::translate("Record", "\346\232\202\345\201\234", nullptr));
        roiRecordBtn->setText(QCoreApplication::translate("Record", "ROI", nullptr));
        exitRecordBtn->setText(QCoreApplication::translate("Record", "\351\200\200\345\207\272", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Record: public Ui_Record {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RECORD_H
