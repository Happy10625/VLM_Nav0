使用方式：

```bash
cd /home/isee601/ws/VLM_Nav

./scripts/start_fixed_map_navigation.sh \
--map /home/isee601/ws/VLM_Nav/shared_maps/lab_ranger_20261006/lab.yaml \
"红色灭火器"
```

 参数会完整透传：

  start_fixed_map_navigation.sh
    → start_navigation_validation.sh
    → system.launch.py map:=...
    → map_server yaml_filename

  更换地图只需修改 --map：

```bash
./scripts/start_fixed_map_navigation.sh \
--map /path/to/new_map.yaml \
--amcl-scan-topic /scan \
"目标描述"
```

启动后，在 RViz 设置 2D Pose Estimate，待 AMCL 收敛且检查通过后手动使能：

```bash
cd /home/isee601/ws/VLM_Nav
#  VLM_NAV_LOCALIZATION_MODE=amcl ./scripts/check_system.sh
./scripts/arm.sh  
```

停止：

```bash
cd /home/isee601/ws/VLM_Nav
./scripts/stop.sh
```





