/****************************************************************************
** Meta object code from reading C++ file 'receivethread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../qt-exipc/receivethread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'receivethread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ReceiveThread_t {
    QByteArrayData data[11];
    char stringdata0[144];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ReceiveThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ReceiveThread_t qt_meta_stringdata_ReceiveThread = {
    {
QT_MOC_LITERAL(0, 0, 13), // "ReceiveThread"
QT_MOC_LITERAL(1, 14, 17), // "signalServoNotice"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 14), // "ServoNotice_t*"
QT_MOC_LITERAL(4, 48, 10), // "servoNotic"
QT_MOC_LITERAL(5, 59, 14), // "signalServoSta"
QT_MOC_LITERAL(6, 74, 16), // "ServoGroupSta_t*"
QT_MOC_LITERAL(7, 91, 8), // "servoSta"
QT_MOC_LITERAL(8, 100, 15), // "signalServoInfo"
QT_MOC_LITERAL(9, 116, 17), // "ServoGroupInfo_t*"
QT_MOC_LITERAL(10, 134, 9) // "servoInfo"

    },
    "ReceiveThread\0signalServoNotice\0\0"
    "ServoNotice_t*\0servoNotic\0signalServoSta\0"
    "ServoGroupSta_t*\0servoSta\0signalServoInfo\0"
    "ServoGroupInfo_t*\0servoInfo"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ReceiveThread[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x06 /* Public */,
       5,    1,   32,    2, 0x06 /* Public */,
       8,    1,   35,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 9,   10,

       0        // eod
};

void ReceiveThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ReceiveThread *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->signalServoNotice((*reinterpret_cast< ServoNotice_t*(*)>(_a[1]))); break;
        case 1: _t->signalServoSta((*reinterpret_cast< ServoGroupSta_t*(*)>(_a[1]))); break;
        case 2: _t->signalServoInfo((*reinterpret_cast< ServoGroupInfo_t*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ReceiveThread::*)(ServoNotice_t * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ReceiveThread::signalServoNotice)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (ReceiveThread::*)(ServoGroupSta_t * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ReceiveThread::signalServoSta)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (ReceiveThread::*)(ServoGroupInfo_t * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ReceiveThread::signalServoInfo)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ReceiveThread::staticMetaObject = { {
    QMetaObject::SuperData::link<QThread::staticMetaObject>(),
    qt_meta_stringdata_ReceiveThread.data,
    qt_meta_data_ReceiveThread,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ReceiveThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ReceiveThread::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ReceiveThread.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int ReceiveThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void ReceiveThread::signalServoNotice(ServoNotice_t * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ReceiveThread::signalServoSta(ServoGroupSta_t * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void ReceiveThread::signalServoInfo(ServoGroupInfo_t * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
