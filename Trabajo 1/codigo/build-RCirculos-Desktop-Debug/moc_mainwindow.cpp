/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../RCirculos/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[18];
    char stringdata0[330];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 23), // "on_btnLoadVideo_clicked"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 22), // "on_btnInitProc_clicked"
QT_MOC_LITERAL(4, 59, 19), // "on_rbCircle_clicked"
QT_MOC_LITERAL(5, 79, 17), // "on_rbRing_clicked"
QT_MOC_LITERAL(6, 97, 10), // "closeEvent"
QT_MOC_LITERAL(7, 108, 12), // "QCloseEvent*"
QT_MOC_LITERAL(8, 121, 5), // "event"
QT_MOC_LITERAL(9, 127, 23), // "on_withDistance_toggled"
QT_MOC_LITERAL(10, 151, 7), // "checked"
QT_MOC_LITERAL(11, 159, 26), // "on_btnLoadFileDist_clicked"
QT_MOC_LITERAL(12, 186, 22), // "on_rbCalibNone_clicked"
QT_MOC_LITERAL(13, 209, 24), // "on_rbCalibOpencv_clicked"
QT_MOC_LITERAL(14, 234, 23), // "on_rbCalibAnkur_clicked"
QT_MOC_LITERAL(15, 258, 22), // "on_rbFrmManual_clicked"
QT_MOC_LITERAL(16, 281, 25), // "on_rbFrmIntervals_clicked"
QT_MOC_LITERAL(17, 307, 22) // "on_rbFrmRansac_clicked"

    },
    "MainWindow\0on_btnLoadVideo_clicked\0\0"
    "on_btnInitProc_clicked\0on_rbCircle_clicked\0"
    "on_rbRing_clicked\0closeEvent\0QCloseEvent*\0"
    "event\0on_withDistance_toggled\0checked\0"
    "on_btnLoadFileDist_clicked\0"
    "on_rbCalibNone_clicked\0on_rbCalibOpencv_clicked\0"
    "on_rbCalibAnkur_clicked\0on_rbFrmManual_clicked\0"
    "on_rbFrmIntervals_clicked\0"
    "on_rbFrmRansac_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x08 /* Private */,
       3,    0,   80,    2, 0x08 /* Private */,
       4,    0,   81,    2, 0x08 /* Private */,
       5,    0,   82,    2, 0x08 /* Private */,
       6,    1,   83,    2, 0x08 /* Private */,
       9,    1,   86,    2, 0x08 /* Private */,
      11,    0,   89,    2, 0x08 /* Private */,
      12,    0,   90,    2, 0x08 /* Private */,
      13,    0,   91,    2, 0x08 /* Private */,
      14,    0,   92,    2, 0x08 /* Private */,
      15,    0,   93,    2, 0x08 /* Private */,
      16,    0,   94,    2, 0x08 /* Private */,
      17,    0,   95,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_btnLoadVideo_clicked(); break;
        case 1: _t->on_btnInitProc_clicked(); break;
        case 2: _t->on_rbCircle_clicked(); break;
        case 3: _t->on_rbRing_clicked(); break;
        case 4: _t->closeEvent((*reinterpret_cast< QCloseEvent*(*)>(_a[1]))); break;
        case 5: _t->on_withDistance_toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_btnLoadFileDist_clicked(); break;
        case 7: _t->on_rbCalibNone_clicked(); break;
        case 8: _t->on_rbCalibOpencv_clicked(); break;
        //case 9: _t->on_rbCalibAnkur_clicked(); break;
        case 10: _t->on_rbFrmManual_clicked(); break;
        //case 11: _t->on_rbFrmIntervals_clicked(); break;
        //case 12: _t->on_rbFrmRansac_clicked(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
