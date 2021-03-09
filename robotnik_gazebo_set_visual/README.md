void Boxscale::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr sdf)
{
  model_ = _parent;
  world_ = model_->GetWorld();

  node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_->Init(world_->GetName());
  pub_visual_ = node_->Advertise<gazebo::msgs::Visual>("~/visual");     

  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
  std::bind(&Boxscale::OnUpdate, this));    

}

void Boxscale::OnUpdate()
{

if(!skip)
{
    scene_ptr= gazebo::rendering::get_scene();
    box_ptr = scene_ptr->GetVisual(model_->GetName());

    if(!box_ptr)
    {   
        std::cout << "Box not loaded" << std::endl;
    }
    else
    {
        iterrations_++;

  if (iterrations_ > 5000)
  {
            ignition::math::Vector3d initial_scale(3.0, 1.0, 1.0);
            pub_visual_->Publish(box.setscale(initial_scale));
        skip = true;
        }
    }
}
}



















____________
______________________________________________________________________________________________
class Boxscale : public gazebo::ModelPlugin
{
    public: void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    public: void OnUpdate();

    class Scalefactor
    {
      public:
        Scalefactor(void) = default;
        Scalefactor(const std::string linkName, gazebo::physics::ModelPtr model)
        {
          model_ = model;
          link_ = model->GetLink(linkName);

          sdf::ElementPtr visualSDF = linkSDF->GetElement("visual");
          visual_name_ = visualSDF->Get<std::string>("name");
        }
        Scalefactor(const Scalefactor&) = default;
        Scalefactor(Scalefactor&&) = default;
        ~Scalefactor(void) = default;

        Scalefactor& operator=(const Scalefactor&) = default;
        Scalefactor& operator=(Scalefactor&&) = default;

        gazebo::msgs::Visual setscale(const ignition::math::Vector3d scale_) const
        {
          gazebo::msgs::Visual visualMsg = link_->GetVisualMessage(visual_name_);
          gazebo::msgs::Vector3d* scale_factor = new gazebo::msgs::Vector3d{gazebo::msgs::Convert(scale_)};

          visualMsg.set_name(link_->GetScopedName());
          visualMsg.set_parent_name(model_->GetScopedName());
          visualMsg.set_allocated_scale(scale_factor);

          return visualMsg;
        }
    }

    private:
      gazebo::rendering::ScenePtr scene_ptr ;
      gazebo::rendering::VisualPtr box_ptr ;
      gazebo::physics::ModelPtr model_;
      gazebo::physics::WorldPtr world_;
      gazebo::event::ConnectionPtr updateConnection;
        skip = false;
        private: size_t iterrations_ = 0;       
        gazebo::transport::NodePtr node_;
        gazebo::transport::PublisherPtr pub_visual_;

        Scalefactor box;

}