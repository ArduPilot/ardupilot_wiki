<p>You can change and check the parameters for another version: <script type="text/javascript" src="../_static/jquery-3.2.1.js">
</script>

<select class="selectpicker" onChange="window.document.location.href=this.options[this.selectedIndex].value;"></select>

<script type="text/javascript">
jQuery.getJSON("../_static/parameters-Rover.json", {}, function(json) {
    $.each(json, function(key, value) {
        $('.selectpicker').append('<option value="' + json[key] + '">' + key + '</option>');
    });
}); </script>   </p>