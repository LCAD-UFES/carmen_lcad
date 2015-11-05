/****************************************************************************
** QLaserDisplay meta object code from reading C++ file 'display-laserdata.h'
**
** Created: Tue Jan 16 07:59:25 2007
**      by: The Qt MOC ($Id: display-laserdata-moc.cpp,v 1.2 2007/01/16 17:48:28 dhaehnel Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "display-laserdata.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *QLaserDisplay::className() const
{
    return "QLaserDisplay";
}

QMetaObject *QLaserDisplay::metaObj = 0;
static QMetaObjectCleanUp cleanUp_QLaserDisplay( "QLaserDisplay", &QLaserDisplay::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString QLaserDisplay::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "QLaserDisplay", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString QLaserDisplay::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "QLaserDisplay", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* QLaserDisplay::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ "value", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"slotChangeValue", 1, param_slot_0 };
    static const QUMethod slot_1 = {"slotJumpToStart", 0, 0 };
    static const QUMethod slot_2 = {"slotRevPlay", 0, 0 };
    static const QUMethod slot_3 = {"slotStop", 0, 0 };
    static const QUMethod slot_4 = {"slotPlay", 0, 0 };
    static const QUMethod slot_5 = {"slotGrid", 0, 0 };
    static const QUMethod slot_6 = {"slotJumpToEnd", 0, 0 };
    static const QUMethod slot_7 = {"slotSpeedFaster", 0, 0 };
    static const QUMethod slot_8 = {"slotSpeedSlower", 0, 0 };
    static const QUMethod slot_9 = {"slotZoomOut", 0, 0 };
    static const QUMethod slot_10 = {"slotZoomIn", 0, 0 };
    static const QUMethod slot_11 = {"slotStepPrev", 0, 0 };
    static const QUMethod slot_12 = {"slotStepNext", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "slotChangeValue(int)", &slot_0, QMetaData::Protected },
	{ "slotJumpToStart()", &slot_1, QMetaData::Protected },
	{ "slotRevPlay()", &slot_2, QMetaData::Protected },
	{ "slotStop()", &slot_3, QMetaData::Protected },
	{ "slotPlay()", &slot_4, QMetaData::Protected },
	{ "slotGrid()", &slot_5, QMetaData::Protected },
	{ "slotJumpToEnd()", &slot_6, QMetaData::Protected },
	{ "slotSpeedFaster()", &slot_7, QMetaData::Protected },
	{ "slotSpeedSlower()", &slot_8, QMetaData::Protected },
	{ "slotZoomOut()", &slot_9, QMetaData::Protected },
	{ "slotZoomIn()", &slot_10, QMetaData::Protected },
	{ "slotStepPrev()", &slot_11, QMetaData::Protected },
	{ "slotStepNext()", &slot_12, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"QLaserDisplay", parentObject,
	slot_tbl, 13,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_QLaserDisplay.setMetaObject( metaObj );
    return metaObj;
}

void* QLaserDisplay::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "QLaserDisplay" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool QLaserDisplay::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: slotChangeValue((int)static_QUType_int.get(_o+1)); break;
    case 1: slotJumpToStart(); break;
    case 2: slotRevPlay(); break;
    case 3: slotStop(); break;
    case 4: slotPlay(); break;
    case 5: slotGrid(); break;
    case 6: slotJumpToEnd(); break;
    case 7: slotSpeedFaster(); break;
    case 8: slotSpeedSlower(); break;
    case 9: slotZoomOut(); break;
    case 10: slotZoomIn(); break;
    case 11: slotStepPrev(); break;
    case 12: slotStepNext(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool QLaserDisplay::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool QLaserDisplay::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool QLaserDisplay::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
