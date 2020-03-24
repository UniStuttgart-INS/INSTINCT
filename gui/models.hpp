#pragma once

#include <QtCore/QObject>
#include <QtWidgets/QLineEdit>

#include <nodes/NodeData>
#include <nodes/NodeDataModel>

#include <memory>

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDataModel;
using QtNodes::PortType;
using QtNodes::PortIndex;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class VectorNavObs : public NodeData
{
  public:
    NodeDataType
        type() const override
    {
        return NodeDataType{ "VectorNavObs",
                             "VectorNavObs" };
    }
};

class UbloxObs : public NodeData
{
  public:
    NodeDataType
        type() const override
    {
        return NodeDataType{ "UbloxObs",
                             "UbloxObs" };
    }
};

//------------------------------------------------------------------------------

/// The model dictates the number of inputs and outputs for the Node
class VectorNavSensor : public NodeDataModel
{
    Q_OBJECT

  public:
    virtual ~VectorNavSensor() {}

  public:
    QString caption() const override
    {
        return QString("VectorNavSensor");
    }

    QString name() const override
    {
        return QString("VectorNavSensor");
    }

  public:
    unsigned int nPorts(PortType portType) const override
    {
        unsigned int result = 1;

        switch (portType)
        {
        case PortType::In:
            result = 0;
            break;

        case PortType::Out:
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
        case PortType::Out:
            switch (portIndex)
            {
            case 0:
                return VectorNavObs().type();
            }
            break;

        case PortType::None:
            break;
        }
        // FIXME: control may reach end of non-void function [-Wreturn-type]
        return NodeDataType();
    }

    std::shared_ptr<NodeData> outData(PortIndex port) override
    {
        return std::make_shared<VectorNavObs>();
    }

    void setInData(std::shared_ptr<NodeData>, int) override
    {
        //
    }

    QWidget* embeddedWidget() override
    {
        return nullptr;
    }
};

/// The model dictates the number of inputs and outputs for the Node
class UbloxSensor : public NodeDataModel
{
    Q_OBJECT

  public:
    UbloxSensor()
        : _lineEdit(new QLineEdit()) {}

    virtual ~UbloxSensor() {}

  public:
    QString caption() const override
    {
        return QString("UbloxSensor");
    }

    QString name() const override
    {
        return QString("UbloxSensor");
    }

  public:
    unsigned int nPorts(PortType portType) const override
    {
        unsigned int result = 1;

        switch (portType)
        {
        case PortType::In:
            result = 0;
            break;

        case PortType::Out:
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
        case PortType::Out:
            switch (portIndex)
            {
            case 0:
                return UbloxObs().type();
            }
            break;

        case PortType::None:
            break;
        }
        // FIXME: control may reach end of non-void function [-Wreturn-type]
        return NodeDataType();
    }

    std::shared_ptr<NodeData> outData(PortIndex port) override
    {
        return std::make_shared<UbloxObs>();
    }

    void setInData(std::shared_ptr<NodeData>, int) override
    {
        //
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
        return _lineEdit;
    }

  private:
    QLineEdit* _lineEdit;
};

/// The model dictates the number of inputs and outputs for the Node
class VectorNavDataLogger : public NodeDataModel
{
    Q_OBJECT

  public:
    virtual ~VectorNavDataLogger() {}

  public:
    QString caption() const override
    {
        return QString("VectorNavDataLogger");
    }

    QString name() const override
    {
        return QString("VectorNavDataLogger");
    }

  public:
    unsigned int nPorts(PortType portType) const override
    {
        unsigned int result = 1;

        switch (portType)
        {
        case PortType::In:
            result = 1;
            break;

        case PortType::Out:
            result = 0;
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
            switch (portIndex)
            {
            case 0:
                return VectorNavObs().type();
            }
            break;
        case PortType::None:
            break;
        }
        // FIXME: control may reach end of non-void function [-Wreturn-type]
        return NodeDataType();
    }

    std::shared_ptr<NodeData> outData(PortIndex port) override
    {
        return nullptr;
    }

    void setInData(std::shared_ptr<NodeData>, int) override
    {
        //
    }

    QWidget* embeddedWidget() override
    {
        return nullptr;
    }
};

/// The model dictates the number of inputs and outputs for the Node
class UbloxDataLogger : public NodeDataModel
{
    Q_OBJECT

  public:
    virtual ~UbloxDataLogger() {}

  public:
    QString caption() const override
    {
        return QString("UbloxDataLogger");
    }

    QString name() const override
    {
        return QString("UbloxDataLogger");
    }

  public:
    unsigned int nPorts(PortType portType) const override
    {
        unsigned int result = 1;

        switch (portType)
        {
        case PortType::In:
            result = 1;
            break;

        case PortType::Out:
            result = 0;
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
            switch (portIndex)
            {
            case 0:
                return UbloxObs().type();
            }
            break;
        case PortType::None:
            break;
        }
        // FIXME: control may reach end of non-void function [-Wreturn-type]
        return NodeDataType();
    }

    std::shared_ptr<NodeData> outData(PortIndex port) override
    {
        return nullptr;
    }

    void setInData(std::shared_ptr<NodeData>, int) override
    {
        //
    }

    QWidget* embeddedWidget() override
    {
        return nullptr;
    }
};
