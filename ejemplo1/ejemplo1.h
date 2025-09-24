#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include "ui_counterDlg.h"

#include "Timer.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
        ejemplo1();

    public slots:
        void doButton();
		void doCount();
		void doSlider(int value);

	private:
		Timer timer;
		int count = 0;


};

#endif // ejemplo1_H
