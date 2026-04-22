//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: qTracer                             #
//#                                                                        #
//#  GNU GPL v2 or later                                                   #
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

	QAction* m_action = nullptr;
};
