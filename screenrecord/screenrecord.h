/**
 * @file screenrecord.h
 * @brief 屏幕录制对话框，配合 ui_record.h。
 *        开始：开始录制，再点一次停止并弹出路径保存 MP4。
 *        暂停：暂停 / 继续。ROI：鼠标划定区域。退出：结束并关闭。
 */
#ifndef SCREENRECORD_H
#define SCREENRECORD_H

#include <QByteArray>
#include <QDialog>
#include <QRect>
#include <QSize>

class QCloseEvent;
class QImage;
class QProcess;
class QTimer;
class RoiPicker;

namespace Ui
{
    class Record;
}

class Record : public QDialog
{
    Q_OBJECT

public:
    explicit Record(QWidget* parent = nullptr);
    ~Record() override;

    virtual void init();
    virtual void reset();
    virtual void deinit();

public slots:
    void start();
    void pause();
    void stop();
    void selectRoi();
    void exitRecord();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    friend class RoiPicker;

    void begin();
    bool finishWriter();
    void saveAs();
    void grabFrame();
    void writeFrame(const QImage& image);
    QImage capture() const;
    void applyRoi(const QRect& roi);
    void cancelRoi();
    void updateButtons();

    Ui::Record* ui;
    RoiPicker* m_picker = nullptr;
    QTimer* m_timer = nullptr;
    QProcess* m_ffmpeg = nullptr;
    QRect m_roi;
    QSize m_size;
    QByteArray m_buffer;
    QString m_tempFile;
    bool m_recording = false;
    bool m_paused = false;
    bool m_grabbing = false;
    bool m_saving = false;
};

#endif
