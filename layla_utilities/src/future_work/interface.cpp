/* Author: Andre Florindo*/

namespace layla_screen_window
{
    StartScreenWidget::StartScreenWidget(QWidget* parent)
    {
        QVBoxLayout* layout = new QVBoxLayout();
        layout->setAlignment(Qt::AlignTop);

        // Top Label Area ---------------------------------------------------
        HeaderWidget* header = new HeaderWidget("LAYLA Deposition Motion Planner", "Specify the location of the file containing the laydown path to perform.", this);
        layout->addWidget(header);

        // Load settings box ---------------------------------------------
        QHBoxLayout* load_files_layout = new QHBoxLayout();
        progress_bar_ = new QProgressBar(this);
        progress_bar_->setMaximum(100);
        progress_bar_->setMinimum(0);
        progress_bar_->hide();
        load_files_layout->addWidget(progress_bar_);
        btn_load_ = new QPushButton("&Load Files", this);
        btn_load_->setMinimumWidth(180);
        btn_load_->setMinimumHeight(40);
        load_files_layout->addWidget(btn_load_);
        load_files_layout->setAlignment(btn_load_, Qt::AlignRight);
        connect(btn_load_, SIGNAL(clicked()), this, SLOT(loadFilesClick()));

        // Next step instructions
        next_label_ = new QLabel(this);
        QFont next_label_font(QFont().defaultFamily(), 11, QFont::Bold);
        next_label_->setFont(next_label_font);
        next_label_->setText("Success! Use the left navigation pane to continue.");
        next_label_->hide();  // only show once the files have been loaded.

        // Final Layout Setup ---------------------------------------------
        // Alignment
        layout->setAlignment(Qt::AlignTop);
        // Verticle Spacer
        QWidget* vspacer = new QWidget(this);
        vspacer->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        layout->addWidget(vspacer);
        // Attach bottom layout
        layout->addWidget(next_label_);
        layout->setAlignment(next_label_, Qt::AlignRight);
        layout->addLayout(load_files_layout);
        this->setLayout(layout);

        this->setWindowTitle("LAYLA Deposition Motion Planner");  // title of window
        QApplication::processEvents();
    }

    void StartScreenWidget::loadFilesClick()
    {
        btn_load_->setDisabled(true);
        progress_bar_->show();

        bool result;

        result = loadExistingFiles();

        // Check if there was a failure loading files
        if (!result)
        {
            btn_load_->setDisabled(false);
            progress_bar_->hide();
        }
    }

    bool StartScreenWidget::loadExistingFiles()
    {   
        std::string path_input = header->getPath();
        
        // Progress Indicator
        progress_bar_->setValue(10);
        QApplication::processEvents();

        // Load the Path file
        if (!getFileContent(path_input, Path Path.vector))            //<-------------  Review
            return false;  // error occured

        // Progress Indicator
        progress_bar_->setValue(50);
        QApplication::processEvents();


        // Test colision, kinematics and if the robot can perform

        // Progress Indicator
        progress_bar_->setValue(100);
        QApplication::processEvents();

        next_label_->show();  // only show once the files have been loaded

        ROS_INFO("Loading Setup Assistant Complete");
        return true;  // success

    }

    bool StartScreenWidget::getFileContent(const std::string& filename, std::vector<double> & newVector)            //<-------------  Review
   {    
        if (filename.empty())
        {
            ROS_ERROR_NAMED("read_data", "Path is empty");
            QMessageBox::warning(this, "Error Loading Files", "No path file specified");
            return false;
        }

        if (!boost::filesystem::exists(filename))
        {
            ROS_ERROR_NAMED("read_data", "File does not exist");
            QMessageBox::warning(this, "Error Loading Files", "Unable to locate the path file");
            return false;
        }

        // Open the File
	    std::ifstream path_file(filename.c_str());
        
	    if(!path_file.good())
	    {
            ROS_ERROR_NAMED("rdf_loader", "Unable to load path");
            QMessageBox::warning(this, "Error Loading Files", "Program could not open the file.\nPlease check console for errors.");
		    return false;
	    }

        

        /*
        std::string buffer;
        path_file.seekg(0, std::ios::end);
        buffer.reserve(path_file.tellg());
        path_file.seekg(0, std::ios::beg);
        buffer.assign((std::istreambuf_iterator<char>(path_file)), std::istreambuf_iterator<char>());
        path_file.close();
        */


        //https://stackoverflow.com/questions/46663046/save-read-double-vector-from-file-c                    //<-------------  Review
        std::vector<char> buffer{};
        std::ifstream ifs(filename, std::ios::in | std::ifstream::binary);
        std::istreambuf_iterator<char> iter(ifs);
        std::istreambuf_iterator<char> end{};
        std::copy(iter, end, std::back_inserter(buffer));
        newVector.reserve(buffer.size() / sizeof(double));
        memcpy(&newVector[0], &buffer[0], buffer.size());
        //Review , only reading a vector, now how to read a matrix 
        // If z is not given in the file, maybe add a collumn of zeros
        return true;
   }


    HeaderWidget::HeaderWidget(const std::string& title, const std::string& instructions, QWidget* parent) : QWidget(parent)
    {
        // Basic widget container
        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->setAlignment(Qt::AlignTop);
        
        // Horizontal layout splitter
        QHBoxLayout* hlayout = new QHBoxLayout();

        // Page Title
        QLabel* page_title = new QLabel(this);
        page_title->setText(title.c_str());
        QFont page_title_font(QFont().defaultFamily(), 18, QFont::Bold);
        page_title->setFont(page_title_font);
        page_title->setWordWrap(true);
        layout->addWidget(page_title);
        layout->setAlignment(page_title, Qt::AlignTop);

        // Page Instructions
        QLabel* page_instructions = new QLabel(this);
        page_instructions->setText(instructions.c_str());
        page_instructions->setWordWrap(true);
        page_instructions->setMinimumWidth(1);
        layout->addWidget(page_instructions);
        layout->setAlignment(page_instructions, Qt::AlignTop);

        // Line Edit for path
        path_box_ = new QLineEdit(this);
        connect(path_box_, SIGNAL(textChanged(QString)), this, SIGNAL(pathChanged(QString)));   // <---- pathChanged
        connect(path_box_, SIGNAL(editingFinished()), this, SIGNAL(pathEditingFinished()));     // <---- pathEditingFinished
        hlayout->addWidget(path_box_);

        // Button
        QPushButton* browse_button = new QPushButton(this);
        browse_button->setText("Browse");
        connect(browse_button, SIGNAL(clicked()), this, SLOT(btnFileDialog()));                  // <---- btnFileDialog Done
        hlayout->addWidget(browse_button);

        // Margin on bottom
        layout->setContentsMargins(0, 0, 0, 0);  // last 15

        // Add horizontal layer to verticle layer
        layout->addLayout(hlayout);

        this->setLayout(layout);
        // this->setSizePolicy( QSizePolicy::Preferred, QSizePolicy::Expanding );
    }

    void HeaderWidget::btnFileDialog()
    {
        QString start_path;

        start_path = path_box_->text();

        if (load_only_)
        {
            path = QFileDialog::getOpenFileName(this, "Open File", start_path, "");
        }
        else
        {
            path = QFileDialog::getSaveFileName(this, "Create/Load File", start_path, "");
        }

        // check they did not press cancel
        if (!path.isNull())
        {
            path_box_->setText(path);
        }
    }


    QString HeaderWidget::getQPath() const
    {
        return path_box_->text();
    }


    std::string HeaderWidget::getPath() const
    {
        return getQPath().toStdString();
    }

    void HeaderWidget::setPath(const QString& path)
    {
        path_box_->setText(path);
    }   

    void HeaderWidget::setPath(const std::string& path)
    {
        path_box_->setText(QString(path.c_str()));
    }


}



int main(int argc, char **argv)
{
    // Start ROS Node                                
    ros::init(argc, argv, "layla_screen_window", ros::init_options::NoSigintHandler);

    // ROS Spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // Create Qt Application
    QApplication qt_app(argc, argv);
    layla_screen_window::SetupAssistantWidget saw(nullptr)
    
    saw.setMinimumWidth(980);
    saw.setMinimumHeight(550);
    saw.show();

    // Wait here until Qt App is finished
    const int result = qt_app.exec();

    // Shutdown ROS
    ros::shutdown();

    return result;
}




