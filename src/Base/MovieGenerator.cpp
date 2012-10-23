/**
   @author Shin'ichiro Nakaoka
*/

#include "MovieGenerator.h"
#include "TimeBar.h"
#include "SceneView.h"
#include "ExtensionManager.h"
#include <cnoid/MenuManager>
#include "Spacer.h"
#include "Archive.h"
#include <cnoid/MainWindow>
#include <gtkmm/box.h>
#include <gtkmm/dialog.h>
#include <gtkmm/entry.h>
#include <gtkmm/spinbutton.h>
#include <gtkmm/checkbutton.h>
#include <gtkmm/messagedialog.h>
#include <glibmm/i18n.h>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

#include "gettext.h"

using namespace std;
using namespace Glib;
using namespace boost;
using namespace cnoid;


namespace {
    
    class MovieGenerator : public Gtk::Dialog
    {
    public:
        MovieGenerator();
        ~MovieGenerator();

        bool store(Archive& archive);
        void restore(const Archive& archive);
        
    protected:
        virtual void on_response(int id);

    private:
        bool isRecording;
        bool requestStopRecording;
        
        Gtk::Entry folderEntry;
        Gtk::Button folderButton;
        Gtk::Entry basenameEntry;
        Gtk::SpinButton beginningTimeSpin;
        Gtk::CheckButton endingTimeCheck;
        Gtk::SpinButton endingTimeSpin;
        Gtk::SpinButton fpsSpin;
        Gtk::SpinButton imageWidthSpin;
        Gtk::SpinButton imageHeightSpin;

        bool doRecordingLoop();
    };
}


void cnoid::initializeMovieGenerator(ExtensionManager& ext)
{
    MovieGenerator* mg = ext.manage(new MovieGenerator());

    MenuManager& mm = ext.menuManager();
    mm.setPath("/Tools");
    mm.addItem(N_("Movie Generator"))
        ->signal_activate().connect(sigc::mem_fun(*mg, &MovieGenerator::show));

    ext.connectProjectArchiver(
        "MovieGenerator", bind(&MovieGenerator::store, mg, _1), bind(&MovieGenerator::restore, mg, _1));
}


MovieGenerator::MovieGenerator()
    : Gtk::Dialog(_("Movie Generator"), MainWindow::instance())
{
    set_modal(false);

    isRecording = false;
    requestStopRecording = false;

    Gtk::VBox* vbox = get_vbox();

    Gtk::HBox* hbox = Gtk::manage(new Gtk::HBox());
    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("Folder"))), Gtk::PACK_SHRINK);
    hbox->pack_start(folderEntry, Gtk::PACK_SHRINK);
    folderEntry.set_width_chars(20);
    folderEntry.set_editable();

    folderButton.set_label(_("Open"));
    hbox->pack_start(folderButton, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("Basename"))), Gtk::PACK_SHRINK);
    basenameEntry.set_text("scene");
    hbox->pack_start(basenameEntry, Gtk::PACK_SHRINK);
    basenameEntry.set_width_chars(20);
    basenameEntry.set_editable();

    hbox = Gtk::manage(new Gtk::HBox());
    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("Begin"))), Gtk::PACK_SHRINK);
    beginningTimeSpin.set_digits(1);
    beginningTimeSpin.set_range(0.0, 9999.9);
    beginningTimeSpin.set_value(0.0);
    beginningTimeSpin.set_increments(0.1, 1.0);
    hbox->pack_start(beginningTimeSpin, Gtk::PACK_SHRINK);

    endingTimeCheck.set_label(_("End"));
    hbox->pack_start(endingTimeCheck, Gtk::PACK_SHRINK);
    
    endingTimeSpin.set_digits(1);
    endingTimeSpin.set_range(0.0, 9999.9);
    endingTimeSpin.set_value(0.0);
    endingTimeSpin.set_increments(0.1, 1.0);
    hbox->pack_start(endingTimeSpin, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label("FPS")), Gtk::PACK_SHRINK);
    fpsSpin.set_digits(1);
    fpsSpin.set_range(1.0, 9999.9);
    fpsSpin.set_value(30.0);
    fpsSpin.set_increments(0.1, 1.0);
    hbox->pack_start(fpsSpin, Gtk::PACK_SHRINK);
    
    hbox = Gtk::manage(new Gtk::HBox());
    vbox->pack_start(*hbox, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label(_("Image size"))), Gtk::PACK_SHRINK);
    
    imageWidthSpin.set_digits(0);
    imageWidthSpin.set_range(1, 9999);
    imageWidthSpin.set_value(640);
    imageWidthSpin.set_increments(1, 10);
    hbox->pack_start(imageWidthSpin, Gtk::PACK_SHRINK);

    hbox->pack_start(*Gtk::manage(new Gtk::Label("x")), Gtk::PACK_SHRINK);

    imageHeightSpin.set_digits(0);
    imageHeightSpin.set_range(1, 9999);
    imageHeightSpin.set_value(480);
    imageHeightSpin.set_increments(1, 10);
    hbox->pack_start(imageHeightSpin, Gtk::PACK_SHRINK);

    show_all_children();
    
    add_button(_("Generate"), Gtk::RESPONSE_OK);
    add_button(_("Cancel / Stop"), Gtk::RESPONSE_CANCEL);
}


MovieGenerator::~MovieGenerator()
{

}


bool MovieGenerator::store(Archive& archive)
{
    archive.write("folder", folderEntry.get_text());
    archive.write("basename", basenameEntry.get_text());
    archive.write("begin", beginningTimeSpin.get_value());
    if(endingTimeCheck.get_active()){
        archive.write("end", endingTimeSpin.get_value());
    }
    archive.write("fps", fpsSpin.get_value());
    archive.write("width", imageWidthSpin.get_value());
    archive.write("heiht", imageHeightSpin.get_value());
    return true;
}


void MovieGenerator::restore(const Archive& archive)
{
    folderEntry.set_text(archive.get("folder", folderEntry.get_text().raw()));
    basenameEntry.set_text(archive.get("basename", basenameEntry.get_text().raw()));
    beginningTimeSpin.set_value(archive.get("begin", beginningTimeSpin.get_value()));
    double endingTime;
    if(archive.read("end", endingTime)){
        endingTimeSpin.set_value(endingTime);
        endingTimeCheck.set_active(true);
    } else {
        endingTimeCheck.set_active(false);
    }
    fpsSpin.set_value(archive.get("fps", fpsSpin.get_value()));
    imageWidthSpin.set_value(archive.get("width", imageWidthSpin.get_value()));
    imageHeightSpin.set_value(archive.get("height", imageHeightSpin.get_value()));
}


void MovieGenerator::on_response(int id)
{
    bool processed = false;
        
    if(id == Gtk::RESPONSE_OK){
        processed = doRecordingLoop();
        if(processed){
            hide();
        }
    } else {
        if(isRecording){
            requestStopRecording = true;
        } else {
            hide();
        }
    }
}


bool MovieGenerator::doRecordingLoop()
{
    requestStopRecording = false;
    isRecording = true;
    int frame = 0;
    double time = beginningTimeSpin.get_value();
    double endingTime = endingTimeCheck.get_active() ? endingTimeSpin.get_value() : std::numeric_limits<double>::max();
    double timeStep = 1.0 / fpsSpin.get_value();
    TimeBar* timeBar = TimeBar::instance();
    SceneView* sceneView = SceneView::mainInstance();
    bool doContinue = true;

    filesystem::path folder(folderEntry.get_text());
    filesystem::path basename(basenameEntry.get_text() + "%08u.png");

    if(!folder.empty()){
        if(filesystem::exists(folder)){
            if(!is_directory(folder)){
                Gtk::MessageDialog dialog(str(format(_("%1% is not a directory.")) % folder),
                                          false, Gtk::MESSAGE_ERROR);
                dialog.run();
                return false;
            }
        } else {
            create_directories(folder);
        }
    }
    
    format filenameFormat((folder / basename).file_string());

    sceneView->setScreenSize(imageWidthSpin.get_value_as_int(), imageHeightSpin.get_value_as_int());

    while(time < endingTime && doContinue){

        doContinue = timeBar->setTime(time);

        while(Glib::MainContext::get_default()->iteration(false)) { };
        if(requestStopRecording){
            break;
        }

        sceneView->saveImage(str(filenameFormat % frame));

        time += timeStep;
        frame++;
    }

    isRecording = false;

    return !requestStopRecording;
}
