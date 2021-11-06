#pragma once

#include <iostream>
#include <ostream>
#include "grasp_detect/util/config_file.h"
#include "grasp_detect/util/parameter.h"

/*
    参数读取
*/
namespace grasp_detect
{
    namespace util
    {

        class ParamsManager
        {

        public:
            ParamsManager(std::string config_name);
            ~ParamsManager();

            int getSelectGraspNum()
            {
                static int num = cfg_file_.getValueOfKey<int>("num_selected", 100);
                return num;
            }

            /**
             * \brief 获得点云预处理相关参数
            */
            const PreprocessCloudParams &getPreprocessCloudParams()
            {
                return preprocess_params_;
            };

            /**
             * \brief 获得夹爪几何参数
            */
            const HandGeometryParams &getHandGeometryParams()
            {
                return hand_geometry_param_;
            };

            /**
             * \brief 获得HandSearch参数
            */
            const HandSearchParams &getHandSearchParams()
            {
                return hand_search_params_;
            };

            /**
             * \brief 获得候选生成参数
            */
            const CandidatesGeneratorParams &getCandidatesGeneratorParams()
            {
                return candidates_params_;
            };

            /**
             * \brief 获得画图参数
            */
            const PlotParams &getPlotParams()
            {
                return plot_param_;
            };

            /**
             * \brief get Clustering Params
            */
            const GraspClusterParams &getGraspClusterParams()
            {
                return grasp_cluster_params_;
            };

            /**
             * \brief get FilterGrasp Params
            */
            const GraspFilterParams &getGraspFilterParams()
            {
                return grasp_filter_params_;
            };

            /**
             * \brief get FilterGrasp Params
            */
            const GraspPredictParams &getGraspPredictParams()
            {
                return grasp_predict_params_;
            };

        private:
            void readPreprocessCloudParamsFromConfig();
            void readHandSearchParamesFromConfig();
            void readHandGeometryParamesFromConfig();
            void readCandidatesParamesFromConfig();
            void readPlotParamesFromConfig();
            void readClusteringParamesConfig();
            void readFlterGraspParamesConfig();
            void readGraspPredictParamesConfig();

        private:
            /* data */
            ConfigFile cfg_file_;
            HandGeometryParams hand_geometry_param_;
            PlotParams plot_param_;
            HandSearchParams hand_search_params_;
            CandidatesGeneratorParams candidates_params_;
            PreprocessCloudParams preprocess_params_;
            GraspClusterParams grasp_cluster_params_;
            GraspFilterParams grasp_filter_params_;
            GraspPredictParams grasp_predict_params_;
        };
    } // namespace util
} // namespace grasp_detect