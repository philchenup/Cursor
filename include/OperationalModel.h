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

#ifndef OPERATIONALMODEL_H
#define OPERATIONALMODEL_H

#include <QAbstractTableModel>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>

class OperationalModel : public QAbstractTableModel
{
	Q_OBJECT
	
public:
	OperationalModel(QObject* parent = nullptr);
	
	virtual ~OperationalModel();
	
	int columnCount(const QModelIndex& parent = QModelIndex()) const;
	
	QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const;
	
	Qt::ItemFlags flags(const QModelIndex &index) const;
	
	QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;
	
	int rowCount(const QModelIndex& parent = QModelIndex()) const;
	
	bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole); 
	
	std::size_t id;
	
	bool stepAxis(int axis, int dir, double stepSize);

	/// 外部输入法兰→TCP 齐次变换 T（旋转+平移）。Identity 表示 TCP 与法兰重合。
	void setToolTransform(const rl::math::Transform& T_flange_tcp);
	const rl::math::Transform& toolTransform() const { return T_flange_tcp; }

	bool displayTcp() const { return showTcp; }

public slots:
	void configurationChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight);

	/// true = 表格显示/编辑 TCP 坐标；false = 法兰坐标。
	void setDisplayTcp(bool tcp);
	
signals:
	void sendConfigurationData(const rl::math::Vector&);

protected:
	
private:
	rl::math::Transform displayedTransform(std::size_t column) const;

	bool IsCartMove = false;
	bool showTcp = false;
	rl::math::Transform T_flange_tcp = rl::math::Transform::Identity();
};

#endif // OPERATIONALMODEL_H
