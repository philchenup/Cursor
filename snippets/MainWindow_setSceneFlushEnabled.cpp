void MainWindow::setSceneFlushEnabled(bool on)
{
	if (!flushSceneTimer) {
		return;
	}
	if (on) {
		flushSceneTimer->start(50);
	}
	else {
		flushSceneTimer->stop();
	}
}
