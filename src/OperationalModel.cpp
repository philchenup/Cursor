//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <QStatusBar>
#include <rl/math/Rotation.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Exception.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/sg/Body.h>

#include "ConfigurationModel.h"
#include "OperationalModel.h"
#include "MainWindow.h"

OperationalModel::OperationalModel(QObject* parent) :
	QAbstractTableModel(parent),
	id(0)
{
}

OperationalModel::~OperationalModel()
{
}

int
OperationalModel::columnCount(const QModelIndex& parent) const
{
	return 1;
}

void
OperationalModel::configurationChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight)
{
	this->beginResetModel();
	this->endResetModel();
}

QVariant
OperationalModel::data(const QModelIndex& index, int role) const
{
	if (nullptr == MainWindow::instance()->mdl)
	{
		return QVariant();
	}
	
	if (!index.isValid())
	{
		return QVariant();
	}

	int num = index.column();

	const rl::math::Transform& transform = MainWindow::instance()->mdl->getOperationalPosition(index.column());
	rl::math::Transform::ConstTranslationPart position = transform.translation();
	rl::math::Vector3 orientation = transform.rotation().eulerAngles(2, 1, 0).reverse();
	
	switch (role)
	{
	case Qt::DisplayRole:
		switch (index.row())
		{
		case 0:
		case 1:
		case 2:
			return QString::number(position(index.row()), 'f', 4) + QString(" mm");
			break;
		case 3:
		case 4:
		case 5:
			return QString::number(orientation(index.row() - 3) * rl::math::RAD2DEG, 'f', 4) + QChar(176);
			break;
		default:
			break;
		}
		break;
	case Qt::EditRole:
		switch (index.row())
		{
		case 0:
		case 1:
		case 2:
			return position(index.row());
			break;
		case 3:
		case 4:
		case 5:
			return orientation(index.row() - 3) * rl::math::RAD2DEG;
			break;
		default:
			break;
		}
		break;
	case Qt::TextAlignmentRole:
		return QVariant(Qt::AlignRight | Qt::AlignVCenter);
		break;
	default:
		break;
	}
	
	return QVariant();
}

Qt::ItemFlags
OperationalModel::flags(const QModelIndex &index) const
{
	if (!index.isValid())
	{
		return Qt::NoItemFlags;
	}
	
	return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
}

QVariant
OperationalModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (nullptr == MainWindow::instance()->mdl)
	{
		return QVariant();
	}
	
	if (Qt::DisplayRole == role && Qt::Vertical == orientation)
	{
		switch (section)
		{
		case 0:
			return "X";
			break;
		case 1:
			return "Y";
			break;
		case 2:
			return "Z";
			break;
		case 3:
			return "A";
			break;
		case 4:
			return "B";
			break;
		case 5:
			return "C";
			break;
		default:
			break;
		}
	}
	
	if (Qt::DisplayRole == role && Qt::Horizontal == orientation)
	{
		return section;
	}
	
	return QVariant();
}

int
OperationalModel::rowCount(const QModelIndex& parent) const
{
	/*if (nullptr == MainWindow::instance()->kinematicModels[this->id])
	{
		return 0;
	}
	
	return MainWindow::instance()->kinematicModels[this->id]->getOperationalDof();*/
	return 6;
}

bool
OperationalModel::stepAxis(int axis, int dir, double stepSize)
{
	if (axis < 0 || axis >= this->rowCount() || dir == 0 || stepSize <= 0.0)
	{
		return false;
	}

	const QModelIndex idx = this->index(axis, 0);
	if (!idx.isValid())
	{
		return false;
	}

	const double current = this->data(idx, Qt::EditRole).toDouble();
	return this->setData(idx, current + dir * stepSize, Qt::EditRole);
}

bool
OperationalModel::setData(const QModelIndex& index, const QVariant& value, int role)
{
	if (nullptr == MainWindow::instance()->mdl)
	{
		return false;
	}
	
	if (index.isValid() && Qt::EditRole == role)
	{
		if (rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(MainWindow::instance()->mdl.get()))
		{
			rl::math::Transform transform = kinematic->getOperationalPosition(index.column());
			rl::math::Vector3 orientation = transform.linear().eulerAngles(2, 1, 0).reverse();
			
			switch (index.row())
			{
			case 0:
			case 1:
			case 2:
				transform.translation()(index.row()) = value.value<rl::math::Real>();
				break;
			case 3:
				transform.linear() = (
					rl::math::AngleAxis(orientation.z(), rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(orientation.y(), rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(value.value<rl::math::Real>() * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
				).toRotationMatrix();
				break;
			case 4:
				transform.linear() = (
					rl::math::AngleAxis(orientation.z(), rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(value.value<rl::math::Real>() * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(orientation.x(), rl::math::Vector3::UnitX())
				).toRotationMatrix();
				break;
			case 5:
				transform.linear() = (
					rl::math::AngleAxis(value.value<rl::math::Real>() * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *
					rl::math::AngleAxis(orientation.y(), rl::math::Vector3::UnitY()) *
					rl::math::AngleAxis(orientation.x(), rl::math::Vector3::UnitX())
				).toRotationMatrix();
				break;
			default:
				break;
			}
			
			rl::math::Vector q = kinematic->getPosition();
			
			std::shared_ptr<rl::mdl::InverseKinematics> ik;
			
			ik = std::make_shared<rl::mdl::JacobianInverseKinematics>(kinematic);
			rl::mdl::JacobianInverseKinematics* jacobianIk = static_cast<rl::mdl::JacobianInverseKinematics*>(ik.get());
			jacobianIk->setMethod(rl::mdl::JacobianInverseKinematics::Method::svd);

			for (std::size_t i = 0; i < kinematic->getOperationalDof(); ++i)
			{
				//ik->goals.push_back(::std::make_pair(i == index.column() ? transform : kinematic->getOperationalPosition(i), i));
				ik->addGoal(i == index.column() ? transform : kinematic->getOperationalPosition(i), i );
			}

			std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
			bool solved = ik->solve();
			std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
			
			if (solved) {
				kinematic->forwardPosition();
				rl::math::Vector q_after = kinematic->getPosition();
				double sum_of_squares = (q_after - q).squaredNorm();
				if (std::sqrt(sum_of_squares) > 20) solved = false;
			}

			if (solved)
			{
				MainWindow::instance()->statusBar()->showMessage("IK solved in " + QString::number(std::chrono::duration<double>(stop - start).count() * rl::math::UNIT2MILLI) + " ms", 2000);
				
				kinematic->forwardPosition();
				
				for (std::size_t i = 0; i < MainWindow::instance()->sceneModel->getNumBodies(); ++i)
				{
					MainWindow::instance()->sceneModel->getBody(i)->setFrame(kinematic->getBodyFrame(i));
				}
				
				emit dataChanged(this->createIndex(0, 0), this->createIndex(this->rowCount(), this->columnCount()));

				return true;
			}
			else
			{
				MainWindow::instance()->statusBar()->showMessage("IK failed", 2000);
				
				kinematic->setPosition(q);
				kinematic->forwardPosition();
			}
		}
	}
	
	return false;
}
