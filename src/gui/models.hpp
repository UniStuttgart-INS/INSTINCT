#pragma once

#include <QtCore/QObject>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QLabel>
#include <QtWidgets/QFormLayout>

#include <nodes/NodeData>
#include <nodes/NodeDataModel>

#define GUI
#include <../NodeInterface.hpp>

#include <memory>

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::PortType;
using QtNodes::PortIndex;

/// The model dictates the number of inputs and outputs for the Node
class NodeDataTemplate : public NodeDataModel
{
    Q_OBJECT

  public:
    NodeDataTemplate() {}

    NodeDataTemplate(QString const& name)
        : _name(name)
    {
        _mainWidget = new QWidget();
        _mainWidget->setStyleSheet("background-color: transparent; color: white");
        QFormLayout* _layout = new QFormLayout(_mainWidget);
        _layout->setContentsMargins(0, 5, 0, 0);
        _layout->setLabelAlignment(Qt::AlignRight);

        _lineEdit = new QLineEdit();
        _lineEdit->setObjectName("Name");
        _lineEdit->setStyleSheet("QLineEdit { background: rgb(220,220,220); selection-background-color: rgb(169,169,169); color: black }");
        _layout->addRow(tr("Name:"), _lineEdit);

        auto lineEdit2 = new QLineEdit();
        _layout->addRow(tr("Emailsasasa:"), lineEdit2);
    }

    // Do not delete objects here, as qt handles it himself
    virtual ~NodeDataTemplate() {}

  public:
    QString caption() const override
    {
        return _name;
    }

    QString name() const override
    {
        return _name;
    }

  public:
    unsigned int nPorts(PortType portType) const override
    {
        unsigned int result = 1;

        switch (portType)
        {
        case PortType::In:
            if (_name == "Integrator" || _name == "VectorNavDataLogger")
                result = 1;
            if (_name == "UbloxSensor" || _name == "VectorNavSensor")
                result = 0;
            break;

        case PortType::Out:
            if (_name == "Integrator" || _name == "VectorNavDataLogger")
                result = 0;
            if (_name == "UbloxSensor" || _name == "VectorNavSensor")
                result = 1;
            break;
        case PortType::None:
            break;
        }

        return result;
    }

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override
    {
        switch (portType)
        {
        case PortType::In:
            if (portIndex == 0)
            {
                if (_name == "Integrator")
                    return { "InsObs", "InsObs" };
                else if (_name == "VectorNavDataLogger")
                    return { "VectorNavObs", "VectorNavObs" };
            }
            break;
        case PortType::Out:
            if (portIndex == 0)
            {
                if (_name == "UbloxSensor")
                    return { "UbloxObs", "UbloxObs" };
                else if (_name == "VectorNavSensor")
                    return { "VectorNavObs", "VectorNavObs" };
            }
            break;
        case PortType::None:
            break;
        }
        return NodeDataType();
    }

    QJsonObject save() const override
    {
        QJsonObject modelJson = NodeDataModel::save();

        modelJson["text"] = _lineEdit->text();

        return modelJson;
    }

    void restore(QJsonObject const& p) override
    {
        QJsonValue v = p["text"];

        if (!v.isUndefined())
        {
            QString strText = v.toString();

            _lineEdit->setText(strText);
        }
    }

    QWidget* embeddedWidget() override
    {
        return _mainWidget;
    }

    std::shared_ptr<NodeData> outData(PortIndex) override
    {
        return nullptr;
    }

    void setInData(std::shared_ptr<NodeData>, int) override {}

  private:
    QString const _name = "Template";
    QWidget* _mainWidget;
    QLineEdit* _lineEdit;
};
