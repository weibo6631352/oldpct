#include "QAligningObjectTemplates.h"
#include <QString>
#include <QStringList>
#include <QFileDialog>
#include <QComboBox>
#include "QRendView.h"
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/conversions.h>
#include <QMessageBox>

class FeatureCloud
{
public:
	// A bit of shorthand
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

	FeatureCloud() :
		search_method_xyz_(new SearchMethod),
		normal_radius_(4.0f),
		feature_radius_(4.0f)
	{}

	~FeatureCloud() {}

	// Process the given cloud
	void
		setInputCloud(PointCloud::Ptr xyz)
	{
			xyz_ = xyz;
			processInput();
		}

	// Load and process the cloud in the given PCD file
	void
		loadInputCloud(const std::string &pcd_file)
	{
			xyz_ = PointCloud::Ptr(new PointCloud);
			pcl::io::loadPCDFile(pcd_file, *xyz_);
			processInput();
		}

	// Get a pointer to the cloud 3D points
	PointCloud::Ptr
		getPointCloud() const
	{
			return (xyz_);
		}

	// Get a pointer to the cloud of 3D surface normals
	SurfaceNormals::Ptr
		getSurfaceNormals() const
	{
			return (normals_);
		}

	// Get a pointer to the cloud of feature descriptors
	LocalFeatures::Ptr
		getLocalFeatures() const
	{
			return (features_);
		}

protected:
	// Compute the surface normals and local features
	void
		processInput()
	{
			computeSurfaceNormals();
			computeLocalFeatures();
		}

	// Compute the surface normals
	void
		computeSurfaceNormals()
	{
			normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
			norm_est.setInputCloud(xyz_);
			norm_est.setSearchMethod(search_method_xyz_);
			norm_est.setRadiusSearch(normal_radius_);
			norm_est.compute(*normals_);
		}

	// Compute the local feature descriptors
	void
		computeLocalFeatures()
	{
			features_ = LocalFeatures::Ptr(new LocalFeatures);

			pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
			fpfh_est.setInputCloud(xyz_);
			fpfh_est.setInputNormals(normals_);
			fpfh_est.setSearchMethod(search_method_xyz_);
			fpfh_est.setRadiusSearch(feature_radius_);
			fpfh_est.compute(*features_);
		}

private:
	// Point cloud data
	PointCloud::Ptr xyz_;
	SurfaceNormals::Ptr normals_;
	LocalFeatures::Ptr features_;
	SearchMethod::Ptr search_method_xyz_;

	// Parameters
	float normal_radius_;
	float feature_radius_;
};

class TemplateAlignment
{
public:

	// A struct for storing alignment results
	struct Result
	{
		float fitness_score;
		Eigen::Matrix4f final_transformation;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	TemplateAlignment() :
		min_sample_distance_(0.1f),
		max_correspondence_distance_(/*0.01f*0.01f*/4),
		nr_iterations_(500)
	{
		// Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
		sac_ia_.setMinSampleDistance(min_sample_distance_);
		sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
		sac_ia_.setMaximumIterations(nr_iterations_);
	}

	~TemplateAlignment() {}

	// Set the given cloud as the target to which the templates will be aligned
	void
		setTargetCloud(FeatureCloud &target_cloud)
	{
			target_ = target_cloud;
			sac_ia_.setInputTarget(target_cloud.getPointCloud());
			sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
		}

	// Add the given cloud to the list of template clouds
	void
		addTemplateCloud(FeatureCloud &template_cloud)
	{
			templates_.push_back(template_cloud);
		}

	// Align the given template cloud to the target specified by setTargetCloud ()
	void
		align(FeatureCloud &template_cloud, TemplateAlignment::Result &result)
	{
			sac_ia_.setInputCloud(template_cloud.getPointCloud());
			sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

			pcl::PointCloud<pcl::PointXYZ> registration_output;
			sac_ia_.align(registration_output);

			result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
			result.final_transformation = sac_ia_.getFinalTransformation();
		}

	// Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
	void
		alignAll(std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
	{
			results.resize(templates_.size());
			for (size_t i = 0; i < templates_.size(); ++i)
			{
				align(templates_[i], results[i]);
			}
		}

	std::vector<Result, Eigen::aligned_allocator<Result> >  findAlignment()
	{
			std::vector<Result, Eigen::aligned_allocator<Result> > results;
			alignAll(results);
			return results;
	}

	// Align all of template clouds to the target cloud to find the one with best alignment score
	int
		findBestAlignment(std::vector<Result, Eigen::aligned_allocator<Result> > results, TemplateAlignment::Result &result)
	{
// 			// Align all of the templates to the target cloud
// 			std::vector<Result, Eigen::aligned_allocator<Result> > results;
// 			alignAll(results);

			// Find the template with the best (lowest) fitness score
			float lowest_score = std::numeric_limits<float>::infinity();
			int best_template = 0;
			for (size_t i = 0; i < results.size(); ++i)
			{
				const Result &r = results[i];
				if (r.fitness_score < lowest_score)
				{
					lowest_score = r.fitness_score;
					best_template = (int)i;
				}
			}

			// Output the best alignment
			result = results[best_template];
			return (best_template);
		}

private:
	// A list of template clouds and the target to which they will be aligned
	std::vector<FeatureCloud> templates_;
	FeatureCloud target_;

	// The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
	float min_sample_distance_;
	float max_correspondence_distance_;
	int nr_iterations_;
};


QAligningObjectTemplates::QAligningObjectTemplates(QWidget *parent)
	: QSubDialogBase(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_2, &QPushButton::clicked, [this](){
		ui.lineEdit->setText(QFileDialog::getOpenFileName(
			this,
			QStringLiteral("Ä£°æÎÄ¼þ"),
			NULL,// ".",
			QStringLiteral("*.pcd")));
	});

}

QAligningObjectTemplates::~QAligningObjectTemplates()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr rgbCloud2Cloud(pcl::PointCloud<PointT>::Ptr input)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > output(new pcl::PointCloud<pcl::PointXYZ>);
	int M = input->points.size();

	for (int i = 0; i < M; i++)
	{
		pcl::PointXYZ p;
		p.x = input->points[i].x;
		p.y = input->points[i].y;
		p.z = input->points[i].z;
		output->points.push_back(p);
	}
	return output;
}

pcl::PointCloud<PointT>::Ptr cloud2rgbCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, uint8_t r, uint8_t g, uint8_t b)
{
	pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
	int M = input->points.size();

	for (int i = 0; i < M; i++)
	{
		PointT p;
		p.x = input->points[i].x;
		p.y = input->points[i].y;
		p.z = input->points[i].z;
		p.r = r;
		p.g = g;
		p.b = b;
		output->points.push_back(p);
	}
	return output;
}

void QAligningObjectTemplates::OnApply()
{
	QRendView* ins = QRendView::MainRendView();
	QString  template_cloud_file = ui.lineEdit->text();

	// Load the object templates specified in the object_templates.txt file
	std::vector<FeatureCloud> object_templates;
	FeatureCloud template_cloud;
	template_cloud.loadInputCloud(template_cloud_file.toLocal8Bit().data());
	object_templates.push_back(template_cloud);
// 	FeatureCloud template_cloud1;
// 	template_cloud1.loadInputCloud(template_cloud_file.toLocal8Bit().data());
// 	object_templates.push_back(template_cloud1);
// 	std::ifstream input_stream(argv[1]);
// 	object_templates.resize(0);
// 	std::string pcd_filename;
// 	while (input_stream.good())
// 	{
// 		std::getline(input_stream, pcd_filename);
// 		if (pcd_filename.empty() || pcd_filename.at(0) == '#') // Skip blank lines or comments
// 			continue;
// 
// 		FeatureCloud template_cloud;
// 		template_cloud.loadInputCloud(pcd_filename);
// 		object_templates.push_back(template_cloud);
// 	}
// 	input_stream.close();

	// Load the target cloud PCD file
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	/*pcl::io::loadPCDFile(argv[2], *cloud);*/

	cloud = rgbCloud2Cloud(PCManage::ins().cloud_);
	//pcl::copyPointCloud(*PCManage::ins().cloud_, cloud);
// 	pcl::fromPCLPointCloud2<pcl::PointXYZ>(*PCManage::ins().cloud_, cloud);
// 	pcl::toPCLPointCloud2()

	// Preprocess the cloud by...
	// ...removing distant points
// 	const float depth_limit = 1.0;
// 	pcl::PassThrough<pcl::PointXYZ> pass;
// 	pass.setInputCloud(cloud);
// 	pass.setFilterFieldName("z");
// 	pass.setFilterLimits(0, depth_limit);
// 	pass.filter(*cloud);

// 	// ... and downsampling the point cloud
// 	const float voxel_grid_size = 0.5f;
// 	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
// 	vox_grid.setInputCloud(cloud);
// 	vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
// 	//vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
// 	vox_grid.filter(*tempCloud);
// 	cloud = tempCloud;

	// Assign to the target FeatureCloud
	FeatureCloud target_cloud;
	target_cloud.setInputCloud(cloud);

	// Set the TemplateAlignment inputs
	TemplateAlignment template_align;
	for (size_t i = 0; i < object_templates.size(); ++i)
	{
		template_align.addTemplateCloud(object_templates[i]);
	}
	template_align.setTargetCloud(target_cloud);

	// Find the best template alignment
	std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<TemplateAlignment::Result> > results = template_align.findAlignment();
	TemplateAlignment::Result best_alignment;
	
	int best_index = template_align.findBestAlignment(results, best_alignment);

	const FeatureCloud &best_template = object_templates[best_index];


	// Save the aligned template for visualization
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_cloud, best_alignment.final_transformation);
	//pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);
	*PCManage::ins().cloud_ += *cloud2rgbCloud(transformed_cloud, 255, 0 , 0);

	for (size_t i = 0; i < results.size(); ++i)
	{
		TemplateAlignment::Result best_alignment = results[i];
		const FeatureCloud &best_template = object_templates[i];

		// Save the aligned template for visualization
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*best_template.getPointCloud(), *transformed_cloud, best_alignment.final_transformation);
		//pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);
		*PCManage::ins().cloud_ += *cloud2rgbCloud(transformed_cloud, 0, 255, 0);
	}



	ins->UpdateView();
	this->accept();
}