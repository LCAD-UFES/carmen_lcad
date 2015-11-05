/****************************************************************************
** Meta object code from reading C++ file 'QtImageFrame.qt.H'
**
** Created: Thu Dec 13 09:35:30 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "QtImageFrame.qt.H"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QtImageFrame.qt.H' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtImageFrame[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      19,   14,   13,   13, 0x09,
      32,   13,   13,   13, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_QtImageFrame[] = {
    "QtImageFrame\0\0zoom\0setZoom(int)\0"
    "saveImage()\0"
};

void QtImageFrame::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QtImageFrame *_t = static_cast<QtImageFrame *>(_o);
        switch (_id) {
        case 0: _t->setZoom((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->saveImage(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QtImageFrame::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QtImageFrame::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_QtImageFrame,
      qt_meta_data_QtImageFrame, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtImageFrame::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtImageFrame::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtImageFrame::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtImageFrame))
        return static_cast<void*>(const_cast< QtImageFrame*>(this));
    return QWidget::qt_metacast(_clname);
}

int QtImageFrame::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
