#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	static int min = 100;
	static int max = 500;

	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );

	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	connect(horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(doSlider(int)) );

	lcdNumber_2->display(min);
	horizontalSlider->setMinimum(min);
	horizontalSlider->setMaximum(max);

	horizontalSlider->setValue(lcdNumber_2->intValue());
	timer.start(horizontalSlider->value());
}

void ejemplo1::doButton()
{
	count = 0;
}

void ejemplo1::doCount()
{
	lcdNumber->display(count++);
}

void ejemplo1::doSlider(int value)
{
	timer.setInterval(value);
	lcdNumber_2->display(value);
}

