/******************************************************************************
*            Model Order Reduction plugin for SOFA                            *
*                         version 1.0                                         *
*                       Copyright © Inria                                     *
*                       All rights reserved                                   *
*                       2018                                                  *
*                                                                             *
* This software is under the GNU General Public License v2 (GPLv2)            *
*            https://www.gnu.org/licenses/licenses.en.html                    *
*                                                                             *
*                                                                             *
*                                                                             *
* Authors: Olivier Goury, Felix Vanneste                                      *
*                                                                             *
* Contact information: https://project.inria.fr/modelorderreduction/contact   *
******************************************************************************/
#ifndef SOFA_COMPONENT_MAPPING_MODELORDERREDUCTIONMAPPING_INL
#define SOFA_COMPONENT_MAPPING_MODELORDERREDUCTIONMAPPING_INL

#include "ModelOrderReductionMapping.h"
#include <sofa/defaulttype/RigidTypes.h>
#include <SofaBaseTopology/GridTopology.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/helper/AdvancedTimer.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <sofa/helper/logging/Messaging.h>
#include "../loader/MatrixLoader.h"

// verify timing
#include <sofa/helper/system/thread/CTime.h>

namespace sofa
{

namespace component
{

namespace mapping
{

using std::vector;
using sofa::component::loader::MatrixLoader;

template<class TIn, class TOut>
void ModelOrderReductionMapping<TIn, TOut>::init()
{
    const unsigned n_in = fromModel->getSize();
    const unsigned n_out = toModel->getSize();
    msg_info(this) << "Number of modes: "<<n_in<<" Number of FEM nodes: "<<n_out;

    MatrixLoader<Eigen::MatrixXd>* matLoader = new MatrixLoader<Eigen::MatrixXd>();
    matLoader->setFileName(d_modesPath.getValue());
    msg_info(this) << "Name of data file read";

    matLoader->load();
    msg_info(this) << "file loaded";

    matLoader->getMatrix(m_modesEigen);
    msg_info(this) << "Matrix Obtained";

    if (m_modesEigen.cols()<n_in)
    {
        msg_error(this) << "Error in number of modes: " << n_in << " asked, but only " << m_modesEigen.cols() << " present in input file: " << d_modesPath.getValue();
    }
    else
    {
        m_modesEigen.conservativeResize(Eigen::NoChange,n_in);
    }
    msg_info(this) << "Nb rows" << m_modesEigen.rows();
    msg_info(this) << "Nb cols" << m_modesEigen.cols();

    delete matLoader;

    Inherit::init();

    msg_info(this) <<"out init";
}

template <class TIn, class TOut>
void ModelOrderReductionMapping<TIn, TOut>::apply(const core::MechanicalParams * /*mparams*/, Data<VecCoord>& dOut, const Data<InVecCoord>& dIn)
{
    helper::WriteOnlyAccessor< Data<VecCoord> > out = dOut;
    helper::ReadAccessor< Data<InVecCoord> > in = dIn;
    sofa::helper::AdvancedTimer::stepBegin("apply in OliviersMapping");
    const Data<VecCoord>*restPos =  toModel->read(core::VecCoordId::restPosition());
    double timeScale, time ;
    sofa::helper::system::thread::CTime *timer;
    timeScale = 1000.0 / (double)sofa::helper::system::thread::CTime::getTicksPerSec();
    time = (double)timer->getTime();
    for(unsigned int i=0; i<out.size(); i++)
    {
        out[i]=restPos->getValue()[i];
        for(unsigned int j=0; j<in.size(); j++)
        {
            InReal alpha= in[j][0];
            out[i] +=Deriv(m_modesEigen(3*i,j),m_modesEigen(3*i+1,j),m_modesEigen(3*i+2,j))*alpha;
        }
    }
    sofa::helper::AdvancedTimer::stepEnd("apply in OliviersMapping");
    msg_info(this) <<" apply : "<<( (double)timer->getTime() - time)*timeScale<<" ms";

}

template <class TIn, class TOut>
void ModelOrderReductionMapping<TIn, TOut>::applyJ(const core::MechanicalParams * /*mparams*/, Data<VecDeriv>& dOut, const Data<InVecDeriv>& dIn)
{
    helper::WriteOnlyAccessor< Data<VecDeriv> > out = dOut;
    helper::ReadAccessor< Data<InVecDeriv> > in = dIn;
    double timeScale, time ;
    sofa::helper::system::thread::CTime *timer;
    timeScale = 1000.0 / (double)sofa::helper::system::thread::CTime::getTicksPerSec();
    time = (double)timer->getTime();
    sofa::helper::AdvancedTimer::stepBegin("applyJ in OliviersMapping");
    Eigen::VectorXd inEig;
    inEig.resize(in.size());
    Eigen::VectorXd outEig;
    outEig.resize(3*out.size());
    for(unsigned int j=0; j<in.size(); j++)
    {
        inEig(j) = in[j][0];
    }
    outEig = m_modesEigen*inEig;
    for(unsigned int i=0; i<out.size(); i++)
    {
        out[i] = Deriv(outEig(3*i),outEig(3*i+1),outEig(3*i+2));
    }
    sofa::helper::AdvancedTimer::stepEnd("applyJ in OliviersMapping");
    msg_info(this) <<" applyJ : "<<( (double)timer->getTime() - time)*timeScale<<" ms";
}

template<class TIn, class TOut>
void ModelOrderReductionMapping<TIn, TOut>::applyJT(const core::MechanicalParams * /*mparams*/, Data<InVecDeriv>& dOut, const Data<VecDeriv>& dIn)
{
    helper::WriteAccessor< Data<InVecDeriv> > out = dOut;
    helper::ReadAccessor< Data<VecDeriv> > in = dIn;
    double timeScale, time ;
    sofa::helper::system::thread::CTime *timer;
    timeScale = 1000.0 / (double)sofa::helper::system::thread::CTime::getTicksPerSec();
    time = (double)timer->getTime();
    sofa::helper::AdvancedTimer::stepBegin("applyJt in ModelOrderReductionMapping");
    Eigen::VectorXd inEig;
    inEig.resize(3*in.size());
    Eigen::VectorXd outEig;
    outEig.resize(out.size());


    for(unsigned int i=0; i<in.size(); i++)
    {
        inEig(3*i) = in[i][0];
        inEig(3*i+1) = in[i][1];
        inEig(3*i+2) = in[i][2];
    }

    outEig = m_modesEigen.transpose()*inEig;

    for(unsigned int j=0; j<out.size(); j++)
    {
        out[j][0] += outEig(j);
    }

    sofa::helper::AdvancedTimer::stepEnd("applyJt in ModelOrderReductionMapping");
    msg_info(this)<<" applyJT : "<<( (double)timer->getTime() - time)*timeScale<<" ms";
}

template <class TIn, class TOut>
void ModelOrderReductionMapping<TIn, TOut>::applyJT(const core::ConstraintParams * /*cparams*/, Data<InMatrixDeriv>& dOut, const Data<MatrixDeriv>& dIn)
{
    double timeScale, time ;
    sofa::helper::system::thread::CTime *timer;
    timeScale = 1000.0 / (double)sofa::helper::system::thread::CTime::getTicksPerSec();
    time = (double)timer->getTime();
    InMatrixDeriv& out = *dOut.beginEdit();
    const MatrixDeriv& in = dIn.getValue();
    typename Out::MatrixDeriv::RowConstIterator rowItEnd = in.end();
    msg_info(this) << "In apply JT constraint";
    size_t nbModes = m_modesEigen.cols();

//    Eigen::SparseMatrix<double> constraintMat, res;
//    std::vector< Eigen::Triplet<double> > tripletList;
//    tripletList.reserve(in.size());

    for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
    {
        typename Out::MatrixDeriv::ColConstIterator colIt = rowIt.begin();
        typename Out::MatrixDeriv::ColConstIterator colItEnd = rowIt.end();

        // Creates a constraints if the input constraint is not empty.
        if (colIt != colItEnd)
        {
            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());

            while (colIt != colItEnd)
            {
                for(unsigned int j=0; j<nbModes; j++)

                {
                    InDeriv data;
                    data[0] = Deriv(m_modesEigen(3*(colIt.index()),j),m_modesEigen(3*(colIt.index())+1,j),m_modesEigen(3*(colIt.index())+2,j))*(colIt.val());
                    o.addCol(j,data);
                }

                ++colIt;
            }


//            while (colIt != colItEnd)
//            {
//                Deriv colitVal = colIt.index();
//                tripletList.push_back(Eigen::Triplet<double>(rowIt.index() , 3 * (colIt.index()) , colitVal[0]));
//                tripletList.push_back(Eigen::Triplet<double>(rowIt.index() , 3 * (colIt.index()) + 1 , colitVal[1]));
//                tripletList.push_back(Eigen::Triplet<double>(rowIt.index() , 3 * (colIt.index()) + 2 , colitVal[2]));
//                ++colIt;
//            }

        }
        else
        {
            std::cout<<"Not implemented #################################"<<std::endl;
        }
//        constraintMat.setFromTriplets(tripletList.begin(), tripletList.end());
//        res = m_modesEigen.transpose().sparseView()*constraintMat;
//        for (typename Out::MatrixDeriv::RowConstIterator rowIt = in.begin(); rowIt != rowItEnd; ++rowIt)
//        {
//            typename In::MatrixDeriv::RowIterator o = out.writeLine(rowIt.index());
//            o.addCol(j,data);
//        }
    }

    dOut.endEdit();
    msg_info(this) << "OUT apply JT constraint";
    msg_info(this) <<" applyJT constraint : "<<( (double)timer->getTime() - time)*timeScale<<" ms";
}




template <class TIn, class TOut>
const sofa::defaulttype::BaseMatrix* ModelOrderReductionMapping<TIn, TOut>::getJ()
{
//    updateJ();
    return &m_J;
}

template <class TIn, class TOut>
const typename ModelOrderReductionMapping<TIn, TOut>::js_type* ModelOrderReductionMapping<TIn, TOut>::getJs()
{
//    updateJ();
    return &m_Js;
}


} // namespace mapping

} // namespace component

} // namespace sofa

#endif


