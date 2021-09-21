import QtQuick 2.6
import QtQuick.Window 2.2
import QtMultimedia 5.9
import QtCharts 2.0
import QtQml 2.0

Window {
    width: 1024
    height: 768
    visible: true
    title: qsTr("Sciton Boson")
    property int x: 0
    property int y: 0
    Image {
        width: 1024
        height: 768
        source: "images/mainGeometricBack.png"
    }
    Rectangle {
        id: videoWindow
        width: 512
        height: 768 / 2
        Camera {
            id: camera
            deviceId: QtMultimedia.availableCameras[0].deviceId
            imageCapture {
                onImageCaptured: {
                    // Show the preview in an Image
                    photoPreview.source = preview
                }
            }
        }
        VideoOutput {
            source: camera
            focus: visible // to receive focus and capture key events when visible
            anchors.fill: parent

            MouseArea {
                anchors.fill: parent
                onClicked: camera.imageCapture.capture()
            }
        }
        Image {
            id: photoPreview
        }
    }
    Rectangle {
        id: chartWindow
        width: 512
        height: 768 / 2
        anchors.left: videoWindow.right
        opacity: .6
        border.color: "blue"
        border.width: 5
        ChartView {
            title: "High Tempture"
            anchors.fill: parent
            antialiasing: true
            theme: ChartView.ChartThemeBlueCerulean
            animationOptions: ChartView.SeriesAnimations

            ValueAxis {
                id: xAxis
                min: 0
                max: 200 > x ? 200 : x + 1
                tickCount: 11
                labelsColor: "#ff0000"
                labelsFont.pointSize: 13
                labelsFont.bold: true
                labelFormat: '%d'
            }
            ValueAxis {
                id: yAxis
                min: 90
                max: 250
                tickCount: 10
                labelsColor: "#ff0000"
                labelsFont.pointSize: 13
                labelsFont.bold: true
                labelFormat: '%d'
            }

            LineSeries {
                name: "High Tempture Series"
                id: series
                axisX: xAxis
                axisY: yAxis
                color: "#000fff"
                width: 3
            }
        }
    }
    Timer {
        id: updateTimer
        interval: 100
        repeat: true
        running: true
        onTriggered: {
            y = Math.random() * (225 - 99) + 99
            console.log("x:" + x + " y:" + y)
            series.append(x, y)
            x++
        }
    }
}
