/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.TXT for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * @file test_eagle_board_import.cpp
 * Test suite for import of Eagle *.brd board files
 */

#include <pcbnew_utils/board_test_utils.h>
#include <pcbnew_utils/board_file_utils.h>
#include <qa_utils/wx_utils/unit_test_utils.h>

#include <pcbnew/pcb_io/eagle/pcb_io_eagle.h>

#include <board.h>
#include <board_design_settings.h>
#include <footprint.h>
#include <pcb_shape.h>
#include <netinfo.h>
#include <pcb_text.h>
#include <pcb_track.h>
#include <zone.h>
#include <geometry/shape_poly_set.h>

#include <map>
#include <vector>
#include <wx/filename.h>


struct EAGLE_BOARD_IMPORT_FIXTURE
{
    EAGLE_BOARD_IMPORT_FIXTURE() {}
};


BOOST_FIXTURE_TEST_SUITE( EagleBoardImport, EAGLE_BOARD_IMPORT_FIXTURE )


/**
 * Verify that vias imported from an Eagle board are assigned to the correct nets.
 *
 * Regression test for https://gitlab.com/kicad/code/kicad/-/issues/21243 and
 * https://gitlab.com/kicad/code/kicad/-/issues/23016
 *
 * The Adafruit AHT20 board has vias on SDA, SCL, GND, VCC, and VDD nets. A previous bug
 * caused netCode to not increment for signals with only polygons (no pads), which shifted
 * all subsequent net assignments.
 */
BOOST_AUTO_TEST_CASE( ViaNetAssignment )
{
    std::string dataPath = KI_TEST::GetPcbnewTestDataDir()
                           + "plugins/eagle/Adafruit-AHT20-PCB/"
                             "Adafruit AHT20 Temperature & Humidity.brd";

    BOOST_REQUIRE_MESSAGE( wxFileName::FileExists( dataPath ),
                           "Test board file not found: " + dataPath );

    PCB_IO_EAGLE eaglePlugin;
    std::unique_ptr<BOARD> board;

    try
    {
        board = eaglePlugin.LoadBoard( dataPath );
    }
    catch( const IO_ERROR& e )
    {
        BOOST_FAIL( "IO_ERROR loading Eagle board: " + e.What().ToStdString() );
    }
    catch( const std::exception& e )
    {
        BOOST_FAIL( std::string( "Exception loading Eagle board: " ) + e.what() );
    }

    BOOST_REQUIRE( board );

    // Collect vias grouped by net name
    std::map<wxString, int> viasPerNet;

    for( PCB_TRACK* track : board->Tracks() )
    {
        if( track->Type() == PCB_VIA_T )
        {
            NETINFO_ITEM* net = track->GetNet();
            BOOST_REQUIRE( net );

            wxString netName = net->GetNetname();
            viasPerNet[netName]++;
        }
    }

    // The Eagle board has vias on these nets (verified from the XML source)
    BOOST_CHECK_GT( viasPerNet[wxT( "SDA" )], 0 );
    BOOST_CHECK_GT( viasPerNet[wxT( "SCL" )], 0 );
    BOOST_CHECK_GT( viasPerNet[wxT( "GND" )], 0 );
    BOOST_CHECK_GT( viasPerNet[wxT( "VCC" )], 0 );
    BOOST_CHECK_GT( viasPerNet[wxT( "VDD" )], 0 );

    // Verify exact counts from the Eagle XML
    BOOST_CHECK_EQUAL( viasPerNet[wxT( "SDA" )], 2 );
    BOOST_CHECK_EQUAL( viasPerNet[wxT( "SCL" )], 2 );
    BOOST_CHECK_EQUAL( viasPerNet[wxT( "GND" )], 3 );
    BOOST_CHECK_EQUAL( viasPerNet[wxT( "VCC" )], 2 );
    BOOST_CHECK_EQUAL( viasPerNet[wxT( "VDD" )], 2 );

    // No via should be unassigned (net code 0)
    BOOST_CHECK_EQUAL( viasPerNet[wxT( "" )], 0 );
}


/**
 * Verify that closed Eagle Milling wire contours become Edge.Cuts while open milling
 * toolpaths remain on Dwgs.User with their original widths. Exercise both board-level and
 * footprint-contained geometry and confirm that the closed contours form board holes.
 */
BOOST_AUTO_TEST_CASE( MillingGeometryClassification )
{
    std::string dataPath = KI_TEST::GetPcbnewTestDataDir()
                           + "plugins/eagle/milling_cutout.brd";

    BOOST_REQUIRE_MESSAGE( wxFileName::FileExists( dataPath ),
                           "Test board file not found: " + dataPath );

    PCB_IO_EAGLE eaglePlugin;
    std::unique_ptr<BOARD> board;

    try
    {
        board = eaglePlugin.LoadBoard( dataPath );
    }
    catch( const IO_ERROR& e )
    {
        BOOST_FAIL( "IO_ERROR loading Eagle board: " + e.What().ToStdString() );
    }

    BOOST_REQUIRE( board );
    BOOST_REQUIRE_EQUAL( board->Footprints().size(), 1u );

    FOOTPRINT* cutout = board->Footprints().front();
    int        edgeSegmentCount = 0;
    int        openMillingPathCount = 0;

    // The footprint contains one closed four-segment contour and one open milling path.
    // Only the closed component should participate in the board outline.
    for( BOARD_ITEM* item : cutout->GraphicalItems() )
    {
        if( item->Type() != PCB_SHAPE_T )
            continue;

        PCB_SHAPE* shape = static_cast<PCB_SHAPE*>( item );

        if( shape->GetLayer() == Edge_Cuts && shape->GetShape() == SHAPE_T::SEGMENT )
            edgeSegmentCount++;

        if( shape->GetLayer() == Dwgs_User && shape->GetShape() == SHAPE_T::SEGMENT )
        {
            openMillingPathCount++;
            BOOST_CHECK_EQUAL( shape->GetWidth(), pcbIUScale.mmToIU( 1 ) );
        }
    }

    BOOST_CHECK_EQUAL( edgeSegmentCount, 4 );
    BOOST_CHECK_EQUAL( openMillingPathCount, 1 );

    edgeSegmentCount = 0;
    openMillingPathCount = 0;

    // Repeat the same classification checks for geometry from the board's <plain> section.
    for( BOARD_ITEM* item : board->Drawings() )
    {
        if( item->Type() != PCB_SHAPE_T )
            continue;

        PCB_SHAPE* shape = static_cast<PCB_SHAPE*>( item );

        if( shape->GetLayer() == Edge_Cuts && shape->GetWidth() == pcbIUScale.mmToIU( 0.1 ) )
            edgeSegmentCount++;

        if( shape->GetLayer() == Dwgs_User )
        {
            openMillingPathCount++;
            BOOST_CHECK_EQUAL( shape->GetWidth(), pcbIUScale.mmToIU( 1.5 ) );
        }
    }

    BOOST_CHECK_EQUAL( edgeSegmentCount, 4 );
    BOOST_CHECK_EQUAL( openMillingPathCount, 1 );

    // Confirm that both closed contours are interpreted as holes, not merely relayered graphics.
    SHAPE_POLY_SET boardOutline;
    BOOST_REQUIRE( board->GetBoardPolygonOutlines( boardOutline, false ) );
    BOOST_REQUIRE_EQUAL( boardOutline.OutlineCount(), 1 );
    BOOST_CHECK_EQUAL( boardOutline.HoleCount( 0 ), 2 );
}


/**
 * Verify that Eagle wire primitives on copper- and via-restrict layers become
 * rule areas with the correct geometry and restrictions.
 */
BOOST_AUTO_TEST_CASE( RestrictWireKeepouts )
{
    std::string dataPath = KI_TEST::GetPcbnewTestDataDir() + "plugins/eagle/restrict_wires.brd";

    BOOST_REQUIRE_MESSAGE( wxFileName::FileExists( dataPath ), "Test board file not found: " + dataPath );

    PCB_IO_EAGLE           eaglePlugin;
    std::unique_ptr<BOARD> board;

    try
    {
        board = eaglePlugin.LoadBoard( dataPath );
    }
    catch( const IO_ERROR& e )
    {
        BOOST_FAIL( "IO_ERROR loading Eagle board: " + e.What().ToStdString() );
    }

    BOOST_REQUIRE( board );
    BOOST_REQUIRE_EQUAL( board->Footprints().size(), 1u );

    auto checkKeepouts = []( const std::vector<ZONE*>& aZones, int aExpectedFrontKeepouts )
    {
        BOOST_REQUIRE_EQUAL( aZones.size(), static_cast<size_t>( aExpectedFrontKeepouts + 2 ) );

        int frontKeepouts = 0;
        int backKeepouts = 0;
        int viaKeepouts = 0;

        for( const ZONE* zone : aZones )
        {
            BOOST_CHECK( zone->GetIsRuleArea() );
            BOOST_CHECK_GT( zone->Outline()->TotalVertices(), 3 );

            if( zone->GetLayerSet() == LSET( { F_Cu } ) )
            {
                frontKeepouts++;
                BOOST_CHECK( zone->GetDoNotAllowZoneFills() );
                BOOST_CHECK( zone->GetDoNotAllowTracks() );
                BOOST_CHECK( zone->GetDoNotAllowPads() );
            }
            else if( zone->GetLayerSet() == LSET( { B_Cu } ) )
            {
                backKeepouts++;
                BOOST_CHECK( zone->GetDoNotAllowZoneFills() );
                BOOST_CHECK( zone->GetDoNotAllowTracks() );
                BOOST_CHECK( zone->GetDoNotAllowPads() );
            }
            else if( zone->GetLayerSet() == LSET::AllCuMask() )
            {
                viaKeepouts++;
                BOOST_CHECK( zone->GetDoNotAllowVias() );
                BOOST_CHECK( !zone->GetDoNotAllowZoneFills() );
                BOOST_CHECK( !zone->GetDoNotAllowTracks() );
                BOOST_CHECK( !zone->GetDoNotAllowPads() );
            }
        }

        BOOST_CHECK_EQUAL( frontKeepouts, aExpectedFrontKeepouts );
        BOOST_CHECK_EQUAL( backKeepouts, 1 );
        BOOST_CHECK_EQUAL( viaKeepouts, 1 );
    };

    // Exercise wires in the board's plain section and in a package. The package's
    // four curved tRestrict wires reproduce the fiducial geometry that exposed the bug.
    checkKeepouts( board->Zones(), 1 );
    checkKeepouts( board->Footprints().front()->Zones(), 4 );
}


/**
 * Verify that Eagle polygon width and design-rule clearances are retained when
 * importing copper zones.  These values control narrow copper connectivity and
 * the size of the gaps left between zone fills and the board edge.
 */
BOOST_AUTO_TEST_CASE( CopperZoneDesignRules )
{
    std::string dataPath = KI_TEST::GetPcbnewTestDataDir()
                           + "plugins/eagle/copper_zone_design_rules.brd";

    BOOST_REQUIRE_MESSAGE( wxFileName::FileExists( dataPath ),
                           "Test board file not found: " + dataPath );

    PCB_IO_EAGLE           eaglePlugin;
    std::unique_ptr<BOARD> board;

    try
    {
        board = eaglePlugin.LoadBoard( dataPath );
    }
    catch( const IO_ERROR& e )
    {
        BOOST_FAIL( "IO_ERROR loading Eagle board: " + e.What().ToStdString() );
    }

    BOOST_REQUIRE( board );

    // mdCopperDimension is Eagle's copper-to-Dimension-layer clearance rule.
    BOOST_CHECK_EQUAL( board->GetDesignSettings().m_CopperEdgeClearance,
                       pcbIUScale.mmToIU( 0.254 ) );
    BOOST_CHECK( board->m_LegacyCopperEdgeClearanceLoaded );

    BOOST_REQUIRE_EQUAL( board->Zones().size(), 1u );
    const ZONE* copperZone = board->Zones().front();

    BOOST_CHECK( !copperZone->GetIsRuleArea() );

    // The imported minimum feature width is the Eagle polygon's full pen width,
    // while a polygon without an explicit isolate value inherits mdWireWire.
    BOOST_CHECK_EQUAL( copperZone->GetMinThickness(), pcbIUScale.mmToIU( 0.254 ) );
    BOOST_REQUIRE( copperZone->GetLocalClearance().has_value() );
    BOOST_CHECK_EQUAL( *copperZone->GetLocalClearance(), pcbIUScale.mmToIU( 0.2032 ) );
}


/**
 * Verify that text justification is correct after importing an Eagle board with rotated
 * footprints and silkscreen text annotations.
 *
 * Regression test for https://gitlab.com/kicad/code/kicad/-/issues/23016
 *
 * The test board has 4-pin connectors (I2C0, I2C1) rotated R90 with pin label text
 * "G  SDA SCL V+" on the top silkscreen. A bug in orientFPText() had dead code
 * (abs(degrees) <= -180, which can never be true) that prevented text justification
 * from being flipped for footprints rotated 180 or 270 degrees. The board also
 * exercises the layer enable fix by having unmapped Eagle layers (53, 54, 100+)
 * whose sentinel values previously caused std::bad_alloc in LSET::set().
 */
BOOST_AUTO_TEST_CASE( TextJustification )
{
    std::string dataPath = KI_TEST::GetPcbnewTestDataDir()
                           + "plugins/eagle/test_eagle_23016/test_eagle.brd";

    BOOST_REQUIRE_MESSAGE( wxFileName::FileExists( dataPath ),
                           "Test board file not found: " + dataPath );

    PCB_IO_EAGLE eaglePlugin;
    std::unique_ptr<BOARD> board;

    try
    {
        board = eaglePlugin.LoadBoard( dataPath );
    }
    catch( const IO_ERROR& e )
    {
        BOOST_FAIL( "IO_ERROR loading Eagle board: " + e.What().ToStdString() );
    }
    catch( const std::exception& e )
    {
        BOOST_FAIL( std::string( "Exception loading Eagle board: " ) + e.what() );
    }

    BOOST_REQUIRE( board );

    // Board should have 13 elements from the Eagle file
    BOOST_CHECK_EQUAL( board->Footprints().size(), 13 );

    // Find the I2C connector footprints and verify reference text justification.
    // I2C0 and I2C1 are smashed but have no NAME attribute, so their Reference text
    // goes through the non-smashed orientFPText() code path. With the footprint at R90
    // and the package text at 0 degrees, the combined angle is 90 degrees,
    // not triggering the justification flip to RIGHT/TOP.
    for( FOOTPRINT* fp : board->Footprints() )
    {
        wxString ref = fp->GetReference();

        if( ref == wxT( "I2C0" ) || ref == wxT( "I2C1" ) )
        {
            PCB_TEXT& refText = fp->Reference();

            BOOST_CHECK_MESSAGE(
                    refText.GetHorizJustify() == GR_TEXT_H_ALIGN_LEFT,
                    ref + " reference horizontal justification should be LEFT, got "
                            + std::to_string( refText.GetHorizJustify() ) );

            BOOST_CHECK_MESSAGE(
                    refText.GetVertJustify() == GR_TEXT_V_ALIGN_BOTTOM,
                    ref + " reference vertical justification should be BOTTOM, got "
                            + std::to_string( refText.GetVertJustify() ) );
        }
    }

    // Verify the plain text pin labels have correct justification.
    // "G  SDA SCL V+" on F.SilkS (Eagle layer 21) should have LEFT/BOTTOM justification
    // with 90-degree rotation so the text reads bottom-to-top matching the pin order.
    int topPinLabelCount = 0;

    for( BOARD_ITEM* item : board->Drawings() )
    {
        if( item->Type() != PCB_TEXT_T )
            continue;

        PCB_TEXT* text = static_cast<PCB_TEXT*>( item );

        if( text->GetText() != wxT( "G  SDA SCL V+" ) )
            continue;

        if( text->GetLayer() != F_SilkS )
            continue;

        topPinLabelCount++;

        BOOST_CHECK_MESSAGE(
                text->GetHorizJustify() == GR_TEXT_H_ALIGN_LEFT,
                "Pin label text horizontal justification should be LEFT" );

        BOOST_CHECK_MESSAGE(
                text->GetVertJustify() == GR_TEXT_V_ALIGN_BOTTOM,
                "Pin label text vertical justification should be BOTTOM" );

        BOOST_CHECK_MESSAGE(
                text->GetTextAngle() == EDA_ANGLE( 90, DEGREES_T ),
                "Pin label text angle should be 90 degrees" );
    }

    // Two instances of "G  SDA SCL V+" on F.SilkS (one per connector)
    BOOST_CHECK_EQUAL( topPinLabelCount, 2 );
}


/**
 * Verify that Eagle user-layer numbers 160-163 and 170-178 map to the expected KiCad
 * layers when importing footprints from a library.
 *
 * Test for https://gitlab.com/kicad/code/kicad/-/issues/18766
 *
 * The test library contains a footprint with wires on Eagle layers 160-161 (existing
 * Eco1/Eco2 mapping), 162 (User.Drawings), 163 (Margin), and 170-178 (User.1-User.9).
 * Each wire is at a distinct Y-offset so we can match which Eagle layer mapped to which
 * KiCad layer by checking the line's position.
 */
BOOST_AUTO_TEST_CASE( UserLayerMapping )
{
    std::string dataPath = KI_TEST::GetPcbnewTestDataDir()
                           + "plugins/eagle/lbr/user-layers-test.lbr";

    BOOST_REQUIRE_MESSAGE( wxFileName::FileExists( dataPath ),
                           "Test library file not found: " + dataPath );

    PCB_IO_EAGLE eaglePlugin;

    std::unique_ptr<FOOTPRINT> fp;

    try
    {
        fp = eaglePlugin.FootprintLoad( dataPath, wxT( "USER_LAYERS_TEST" ), false, nullptr );
    }
    catch( const IO_ERROR& e )
    {
        BOOST_FAIL( "IO_ERROR loading Eagle footprint: " + e.What().ToStdString() );
    }
    catch( const std::exception& e )
    {
        BOOST_FAIL( std::string( "Exception loading Eagle footprint: " ) + e.what() );
    }

    BOOST_REQUIRE( fp );

    // Each wire in the test library is on a unique Eagle layer at a unique Y-offset (mm).
    // Wires are 10mm long (x=-5 to x=5) and spaced 2mm apart starting at y=5mm.
    struct LAYER_CHECK
    {
        int          yOffsetNm;  // Y-offset in nanometers
        PCB_LAYER_ID expectedLayer;
        const char*  description;
    };

    // Y is negated because KiCad's Y axis is inverted relative to Eagle's
    // clang-format off
    std::vector<LAYER_CHECK> checks = {
        {  -5000000, Eco1_User,  "Eagle 160 -> Eco1.User" },
        {  -7000000, Eco2_User,  "Eagle 161 -> Eco2.User" },
        {  -9000000, Dwgs_User,  "Eagle 162 -> Dwgs.User" },
        { -11000000, Margin,     "Eagle 163 -> Margin" },
        { -13000000, User_1,     "Eagle 170 -> User.1" },
        { -15000000, User_2,     "Eagle 171 -> User.2" },
        { -17000000, User_3,     "Eagle 172 -> User.3" },
        { -19000000, User_4,     "Eagle 173 -> User.4" },
        { -21000000, User_5,     "Eagle 174 -> User.5" },
        { -23000000, User_6,     "Eagle 175 -> User.6" },
        { -25000000, User_7,     "Eagle 176 -> User.7" },
        { -27000000, User_8,     "Eagle 177 -> User.8" },
        { -29000000, User_9,     "Eagle 178 -> User.9" },
    };
    // clang-format on

    // Collect all segment shapes keyed by Y midpoint (exact nm).
    // Test wires are horizontal and at integer mm offsets, so midpoints are exact.
    std::map<int, PCB_LAYER_ID> linesByY;

    for( BOARD_ITEM* item : fp->GraphicalItems() )
    {
        if( item->Type() != PCB_SHAPE_T )
            continue;

        PCB_SHAPE* shape = static_cast<PCB_SHAPE*>( item );

        if( shape->GetShape() != SHAPE_T::SEGMENT )
            continue;

        int yMid = ( shape->GetStart().y + shape->GetEnd().y ) / 2;

        // A duplicate Y would let a stray segment mask a mis-imported wire.
        BOOST_CHECK_MESSAGE( linesByY.emplace( yMid, shape->GetLayer() ).second,
                             "Duplicate segment at y=" + std::to_string( yMid ) + "nm" );
    }

    BOOST_CHECK_EQUAL( linesByY.size(), checks.size() );

    for( const LAYER_CHECK& check : checks )
    {
        auto it = linesByY.find( check.yOffsetNm );

        BOOST_CHECK_MESSAGE( it != linesByY.end(),
                             std::string( check.description )
                                     + " - no line found at y=" + std::to_string( check.yOffsetNm )
                                     + "nm" );

        if( it != linesByY.end() )
        {
            BOOST_CHECK_MESSAGE( it->second == check.expectedLayer,
                                 std::string( check.description )
                                         + " - expected layer " + std::to_string( check.expectedLayer )
                                         + " but got " + std::to_string( it->second ) );
        }
    }
}


BOOST_AUTO_TEST_SUITE_END()
