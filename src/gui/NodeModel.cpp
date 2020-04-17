#include "NodeModel.hpp"

#include <QtWidgets/QLabel>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QComboBox>

#define GUI
#include <../NodeInterface.hpp>

#include <tuple>
#include <iostream>
#include <sstream>

NodeModel::NodeModel() {}

NodeModel::NodeModel(QString const& name)
    : _name(name)
{
    std::cout << "Creating " << name.toStdString() << std::endl;
    _mainWidget = new QWidget();
    _mainWidget->setStyleSheet("background-color: transparent; color: white");
    QFormLayout* _layout = new QFormLayout(_mainWidget);
    _layout->setContentsMargins(0, 5, 0, 0);
    _layout->setLabelAlignment(Qt::AlignRight);

    auto& nodeInterface = NAV::nodeInterfaces.at(name.toStdString());

    for (size_t i = 0; i < nodeInterface.config.size(); i++)
    {
        QString description = QString::fromStdString(std::get<1>(nodeInterface.config.at(i)));

        if (std::get<0>(nodeInterface.config.at(i)) == NAV::NodeInterface::ConfigOptions::CONFIG_BOOL)
        {
            widgets.push_back(new QCheckBox());
            QCheckBox* checkBox = static_cast<QCheckBox*>(widgets.at(widgets.size() - 1));

            checkBox->setChecked(std::stoi(std::get<2>(nodeInterface.config.at(i))));
            checkBox->setObjectName(description);
            checkBox->setProperty("type", std::get<0>(nodeInterface.config.at(i)));
            checkBox->setStyleSheet("QCheckBox::indicator:unchecked { border: 1px solid rgb(220,220,220); }");
            _layout->addRow(description, checkBox);
        }
        else if (std::get<0>(nodeInterface.config.at(i)) == NAV::NodeInterface::ConfigOptions::CONFIG_INT)
        {
            widgets.push_back(new QSpinBox());
            QSpinBox* spinBox = static_cast<QSpinBox*>(widgets.at(widgets.size() - 1));

            std::stringstream lineStream(std::get<2>(nodeInterface.config.at(i)));
            std::string cell;
            // Split line at separator
            for (size_t i = 0; i < 3; i++)
            {
                std::getline(lineStream, cell, '|');
                if (i == 0)
                    spinBox->setMinimum(std::stoi(cell));
                else if (i == 1)
                    spinBox->setValue(std::stoi(cell));
                else if (i == 2)
                    spinBox->setMaximum(std::stoi(cell));
            }

            spinBox->setSingleStep(1);
            spinBox->setObjectName(description);
            spinBox->setProperty("type", std::get<0>(nodeInterface.config.at(i)));
            spinBox->setStyleSheet("QSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
            _layout->addRow(description, spinBox);
        }
        else if (std::get<0>(nodeInterface.config.at(i)) == NAV::NodeInterface::ConfigOptions::CONFIG_FLOAT)
        {
            widgets.push_back(new QDoubleSpinBox());
            QDoubleSpinBox* doubleSpinBox = static_cast<QDoubleSpinBox*>(widgets.at(widgets.size() - 1));

            std::stringstream lineStream(std::get<2>(nodeInterface.config.at(i)));
            std::string cell;
            // Split line at separator
            for (size_t i = 0; i < 3; i++)
            {
                std::getline(lineStream, cell, '|');
                if (i == 0)
                    doubleSpinBox->setMinimum(std::stod(cell));
                else if (i == 1)
                    doubleSpinBox->setValue(std::stod(cell));
                else if (i == 2)
                    doubleSpinBox->setMaximum(std::stod(cell));
            }

            doubleSpinBox->setSingleStep(1.0);
            doubleSpinBox->setObjectName(description);
            doubleSpinBox->setProperty("type", std::get<0>(nodeInterface.config.at(i)));
            doubleSpinBox->setStyleSheet("QDoubleSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
            _layout->addRow(description, doubleSpinBox);
        }
        else if (std::get<0>(nodeInterface.config.at(i)) == NAV::NodeInterface::ConfigOptions::CONFIG_STRING)
        {
            widgets.push_back(new QLineEdit());
            QLineEdit* lineEdit = static_cast<QLineEdit*>(widgets.at(widgets.size() - 1));

            lineEdit->setObjectName(description);
            lineEdit->setProperty("type", std::get<0>(nodeInterface.config.at(i)));
            lineEdit->setStyleSheet("QLineEdit { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
            lineEdit->setText(QString::fromStdString(std::get<2>(nodeInterface.config.at(i))));
            _layout->addRow(description, lineEdit);
        }
        else if (std::get<0>(nodeInterface.config.at(i)) == NAV::NodeInterface::ConfigOptions::CONFIG_LIST)
        {
            widgets.push_back(new QComboBox());
            QComboBox* comboBox = static_cast<QComboBox*>(widgets.at(widgets.size() - 1));

            std::stringstream lineStream(std::get<2>(nodeInterface.config.at(i)));
            std::string cell;
            // Split line at separator
            while (std::getline(lineStream, cell, '|'))
            {
                if (cell.at(0) == '[')
                {
                    cell = cell.substr(1, cell.size() - 2);
                    comboBox->addItem(QString::fromStdString(cell));
                    comboBox->setCurrentIndex(comboBox->count() - 1);
                }
                else
                    comboBox->addItem(QString::fromStdString(cell));
            }
            comboBox->setObjectName(description);
            comboBox->setProperty("type", std::get<0>(nodeInterface.config.at(i)));
            comboBox->setStyleSheet("QComboBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
            _layout->addRow(description, comboBox);
        }
        else if (std::get<0>(nodeInterface.config.at(i)) == NAV::NodeInterface::ConfigOptions::CONFIG_MAP_INT)
        {
            widgets.push_back(new QSpinBox());
            QSpinBox* spinBox = static_cast<QSpinBox*>(widgets.at(widgets.size() - 1));

            std::stringstream lineStream(std::get<2>(nodeInterface.config.at(i)));
            std::string cell;
            // Split line at separator
            for (size_t i = 0; i < 4; i++)
            {
                std::getline(lineStream, cell, '|');
                if (i == 0)
                    spinBox->setProperty("key", QString::fromStdString(cell));
                if (i == 1)
                    spinBox->setMinimum(std::stoi(cell));
                else if (i == 2)
                    spinBox->setValue(std::stoi(cell));
                else if (i == 3)
                    spinBox->setMaximum(std::stoi(cell));
            }

            spinBox->setSingleStep(1);
            spinBox->setObjectName(description);
            spinBox->setProperty("type", std::get<0>(nodeInterface.config.at(i)));
            spinBox->setStyleSheet("QSpinBox { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
            _layout->addRow(description, spinBox);
        }
    }
}

NodeModel::~NodeModel() {}

QString NodeModel::caption() const
{
    return _name;
}

QString NodeModel::name() const
{
    return _name;
}

unsigned int NodeModel::nPorts(PortType portType) const
{
    auto& nodeInterface = NAV::nodeInterfaces.at(_name.toStdString());

    if (portType == PortType::In)
        return static_cast<unsigned int>(nodeInterface.in.size());
    else if (portType == PortType::Out)
        return static_cast<unsigned int>(nodeInterface.out.size());
    else
        return 0;
}

NodeDataType NodeModel::dataType(PortType portType, PortIndex portIndex) const
{
    auto& nodeInterface = NAV::nodeInterfaces.at(_name.toStdString());

    if (portType == PortType::In)
        return { QString::fromStdString(nodeInterface.in.at(static_cast<size_t>(portIndex)).type),
                 QString::fromStdString(nodeInterface.in.at(static_cast<size_t>(portIndex)).type) };
    else if (portType == PortType::Out)
        return { QString::fromStdString(nodeInterface.out.at(static_cast<size_t>(portIndex))),
                 QString::fromStdString(nodeInterface.out.at(static_cast<size_t>(portIndex))) };
    else
        return NodeDataType();
}

QJsonObject NodeModel::save() const
{
    QJsonObject modelJson = NodeDataModel::save();

    for (size_t i = 0; i < widgets.size(); i++)
    {
        if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_BOOL)
            modelJson[widgets.at(i)->objectName()] = static_cast<QCheckBox*>(widgets.at(i))->isChecked();
        else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_INT)
            modelJson[widgets.at(i)->objectName()] = static_cast<QSpinBox*>(widgets.at(i))->value();
        else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_FLOAT)
            modelJson[widgets.at(i)->objectName()] = static_cast<QDoubleSpinBox*>(widgets.at(i))->value();
        else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_STRING)
            modelJson[widgets.at(i)->objectName()] = static_cast<QLineEdit*>(widgets.at(i))->text();
        else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_LIST)
            modelJson[widgets.at(i)->objectName()] = static_cast<QComboBox*>(widgets.at(i))->currentText();
        else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_MAP_INT)
            modelJson[widgets.at(i)->objectName()] = static_cast<QSpinBox*>(widgets.at(i))->value();
    }

    return modelJson;
}

void NodeModel::restore(QJsonObject const& p)
{
    for (size_t i = 0; i < widgets.size(); i++)
    {
        QJsonValue v = p[widgets.at(i)->objectName()];
        if (!v.isUndefined())
        {
            if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_BOOL)
                static_cast<QCheckBox*>(widgets.at(i))->setChecked(v.toBool());
            else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_INT)
                static_cast<QSpinBox*>(widgets.at(i))->setValue(v.toInt());
            else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_FLOAT)
                static_cast<QDoubleSpinBox*>(widgets.at(i))->setValue(v.toDouble());
            else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_STRING)
                static_cast<QLineEdit*>(widgets.at(i))->setText(v.toString());
            else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_LIST)
                static_cast<QComboBox*>(widgets.at(i))->setCurrentText(v.toString());
            else if (widgets.at(i)->property("type").toUInt() == NAV::NodeInterface::ConfigOptions::CONFIG_MAP_INT)
                static_cast<QSpinBox*>(widgets.at(i))->setValue(v.toInt());
        }
    }
}

QWidget* NodeModel::embeddedWidget()
{
    return _mainWidget;
}

std::shared_ptr<NodeData> NodeModel::outData(PortIndex)
{
    return nullptr;
}

void NodeModel::setInData(std::shared_ptr<NodeData>, int) {}