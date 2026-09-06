//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qTracer                       #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: Chia-Chi (Jate) Chiu, NTUT                  #
//#                                                                        #
//##########################################################################

#pragma once

#include "ccStdPluginInterface.h"

//! qTracer: one-shot DFN fracture trace extraction pipeline.
/** Select a point cloud, open the combined parameter dialog, and run
 *  Compute Eigen -> DBSCAN -> Create Traces -> Trace Clustering -> Plane Fitting
 *  in a single go. Intermediate and final groups are added to the DB tree.
 */
class qTracer : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qTracer" FILE "../info.json" )

public:
	explicit qTracer( QObject* parent = nullptr );
	~qTracer() override = default;

	void             onNewSelection( const ccHObject::Container& selectedEntities ) override;
	QList<QAction*>  getActions() override;

private:
	void doPipeline();
	void doColorFilter();
	void doOutcropArea();
	void doP21();
	void doTraceFilter();

	void finalizePipelineResult(class ccHObject* root,
	                            class ccPointCloud* pc,
	                            class ccHObject* facetsGroup,
	                            class ccHObject* tracesGroup,
	                            class ccHObject* planesGroup,
	                            class ccHObject* mergedGroup,
	                            int firstStage);

	QAction* m_action            = nullptr;
	QAction* m_colorFilterAction = nullptr;
	QAction* m_outcropAreaAction = nullptr;
	QAction* m_p21Action         = nullptr;
	QAction* m_traceFilterAction = nullptr;
};
