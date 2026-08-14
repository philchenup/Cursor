// 放到 MainWindow 里 operationalModel 创建完成之后。
// Designer 中两个 QRadioButton 放进同一 QButtonGroup（互斥）：
//   radioButtonFlange  —— 法兰
//   radioButtonTcp     —— TCP
// 选中 TCP 时 toggled(true)，改选法兰时 TCP 会 toggled(false)。

#include <QRadioButton>

QObject::connect(ui->radioButtonTcp, &QRadioButton::toggled,
	operationalModel, &OperationalModel::setDisplayTcp);

// 启动时与默认法兰显示对齐
ui->radioButtonFlange->setChecked(true);
operationalModel->setDisplayTcp(false);
