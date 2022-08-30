/*
Script for downloading and displaying User Alerts in the common-user-alerts.rst page
Requires a table with the css class "useralerts-list"
Written by Stephen Dade (stephen_dade@hotmail.com)
[copywiki destination="copter,plane,rover,antennatracker"]
*/
$(document).ready(function() {
    $.getScript("https://firmware.ardupilot.org/useralerts/manifest.js", function() {
    // figure out which vehicle's wiki we are in
    var url = window.location.href;
    if (url.search("copter") !== -1) {
        var vehicle = 'copter';
    }
    if (url.search("plane") !== -1) {
        var vehicle = 'plane';
    }
    if (url.search("rover") !== -1) {
        var vehicle = 'rover';
    }
    if (url.search("antennatracker") !== -1) {
        var vehicle = 'tracker';
    }
    var content = '';
    //go through all the user alerts and generate table rows
    for (var key in userAlerts) {
            for (var i = 0, len = userAlerts[key].affectedFirmware.length; i < len; i++) {
                //console.log(userAlerts[key].affectedFirmware[i]);
              if (userAlerts[key].affectedFirmware[i] === "all" || userAlerts[key].affectedFirmware[i] === vehicle) {
                    content += '<tr id="' + key + '">';
                    content += '<td>' + key.slice(0, -5) + '</td>';
                    
                    if (userAlerts[key].criticality == "1") {
                        content += '<td>Critical (1)</td>';
                    }
                    else if (userAlerts[key].criticality == "2") {
                        content += '<td>Critical (2)</td>';
                    }
                    else if (userAlerts[key].criticality == "3") {
                        content += '<td>Major (3)</td>';
                    }
                    else if (userAlerts[key].criticality == "4") {
                        content += '<td>Minor (4)</td>';
                    }
                    else {
                        content += '<td>Unknown (' + userAlerts[key].criticality + ')</td>';
                    }
                    
                    content += '<td>' + ((userAlerts[key].hardwareLimited.length === 0 ) ? 'all' : userAlerts[key].hardwareLimited) + '</td>';

                    //figure out versions affected. 4 combinations
                    var versionAffected = ''
                    // all versions, no fix
                    if (userAlerts[key].versionFrom[vehicle] === undefined &&
                    userAlerts[key].versionFixed[vehicle] === undefined) {
                        versionAffected = 'All';
                    }
                    // some versions, no fix
                    else if (userAlerts[key].versionFrom[vehicle] !== undefined &&
                    userAlerts[key].versionFixed[vehicle] === undefined) {
                        versionAffected = userAlerts[key].versionFrom[vehicle] + ' and above';
                    }
                    // all versions, there is a fix
                    else if (userAlerts[key].versionFrom[vehicle] === undefined &&
                    userAlerts[key].versionFixed[vehicle] !== undefined) {
                        versionAffected = 'All versions < ' + userAlerts[key].versionFixed[vehicle];
                    }
                    // some versions, there is a fix
                    else if (userAlerts[key].versionFrom[vehicle] !== undefined &&
                    userAlerts[key].versionFixed[vehicle] !== undefined) {
                        versionAffected = userAlerts[key].versionFrom[vehicle] + ' to < ' + userAlerts[key].versionFixed[vehicle];
                    }
                    content += '<td>' + versionAffected + '</td>';

                    content += '<td>' + userAlerts[key].description + '</td>';
                    // if User Alert is resolved - suggest upgrade instead
                    if (userAlerts[key].versionFixed[vehicle] === undefined) {
                        content += '<td>' + userAlerts[key].mitigation + '</td>';
                    }
                    else {
                        content += '<td>' + 'Upgrade to version ' + userAlerts[key].versionFixed[vehicle] +'</td>';
                    }
                    //figure out status - Open, patched in master, released in stable
                    if (userAlerts[key].versionFixed[vehicle] === undefined) {
                        if (userAlerts[key].fixCommit.length === 0) {
                            content += '<td>Not fixed</td>';
                        }
                        else {
                            content += '<td>Fixed in master, not released</td>';
                        }
                    }
                    else {
                        content += '<td>' + 'Fixed in version ' + userAlerts[key].versionFixed[vehicle] + '</td>';
                    }
                    content += '</tr>';
                }
            }
        }
        // dump the generated table into the tbody
        $('.useralerts-list tbody').html(content);
        // set styling
        $('.useralerts-list tbody td').css('white-space', 'normal');
        $('.useralerts-list thead th').css('white-space', 'normal');
    });
});
