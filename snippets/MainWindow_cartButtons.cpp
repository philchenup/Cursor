// Paste into MainWindow after operationalModel / operationalView setup
// and after the existing signal connections.
//
// Effect: X/Y/Z/A/B/C +/- buttons call OperationalModel::setData the same way
// the QTableView spinbox up/down buttons do (IK + configuration sync via dataChanged).

struct CartBtn { QPushButton* btn; int axis; int dir; };
const CartBtn cartItems[] = {
	{ ui->XSubBtn, 0, -1 }, { ui->XAddBtn, 0, +1 },
	{ ui->YSubBtn, 1, -1 }, { ui->YAddBtn, 1, +1 },
	{ ui->ZSubBtn, 2, -1 }, { ui->ZAddBtn, 2, +1 },
	{ ui->ASubBtn, 3, -1 }, { ui->AAddBtn, 3, +1 },
	{ ui->BSubBtn, 4, -1 }, { ui->BAddBtn, 4, +1 },
	{ ui->CSubBtn, 5, -1 }, { ui->CAddBtn, 5, +1 },
};

auto stepCart = [this, operationalModel](int axis, int dir) {
	if (!operationalModel) {
		return;
	}

	// Match OperationalDelegate: step comes from transStepSpinbox for all XYZABC.
	// If you prefer separate rotation step for ABC, use:
	//   const double step = (axis < 3)
	//       ? ui->transStepSpinbox->value()
	//       : ui->rotateStepSpinbox->value();
	const double step = ui->transStepSpinbox->value();
	operationalModel->stepAxis(axis, dir, step);
};

for (const CartBtn& item : cartItems) {
	QObject::connect(item.btn, &QPushButton::clicked, this, [stepCart, item]() {
		stepCart(item.axis, item.dir);
	});
}
