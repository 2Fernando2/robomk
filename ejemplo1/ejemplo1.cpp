#include "ejemplo1.h"

ejemplo1::ejemplo1(): Ui_Counter()
{
	static int sliderValue = 0;

	setupUi(this);
	show();
	connect(button, SIGNAL(clicked()), this, SLOT(doButton()) );

	connect(&timer, SIGNAL(timeout()), this, SLOT(doCount()) );
	timer.start(sliderValue);

	connect(&slider, SIGNAL(valueChanged(int)), this, SLOT(doSlider()) );
}

void ejemplo1::doButton()
{
	qDebug() << "click on button";
}

void ejemplo1::doCount()
{
	static int count = 0;
	lcdNumber->display(count++);
}

void ejemplo1::doSlider()
{
	timer.interval().setInterval(slider.tickInterval());
}

