// Parser-only check: deliberately never initialize ROS or a DomainBridge instance.
#include <domain_bridge/parse_domain_bridge_yaml_config.hpp>
#include <iostream>
#include <stdexcept>
int main(int argc,char**argv) {
  try {
    for(int i=1;i<argc;++i) {
      const auto config=domain_bridge::parse_domain_bridge_yaml_config(argv[i]);
      if(config.topics.size()!=5) throw std::runtime_error("five task routes required");
      const bool go2=std::string(argv[i]).find("bridge_go2_")!=std::string::npos;
      for(const auto &pair:config.topics) {
        const auto &topic=pair.first;
        const auto from=pair.second.reversed()?topic.to_domain_id:topic.from_domain_id;
        const auto to=pair.second.reversed()?topic.from_domain_id:topic.to_domain_id;
        std::cout<<topic.topic_name<<' '<<from<<"->"<<to<<' '<<topic.type_name<<'\n';
        const bool source_go2=topic.topic_name.find("/multi_agent/go2/")==0;
        if(topic.type_name.find("multi_agent_task_msgs/msg/")!=0 ||
           pair.second.bidirectional() || from!=(go2==source_go2 ? 0u:40u) ||
           to!=(go2==source_go2 ? 40u:0u))
          throw std::runtime_error("source-PC route/type mismatch");
      }
      std::cout<<argv[i]<<": five directed routes parsed\n";
    }
  } catch(const std::exception&e) {std::cerr<<e.what()<<'\n';return 1;}
}
