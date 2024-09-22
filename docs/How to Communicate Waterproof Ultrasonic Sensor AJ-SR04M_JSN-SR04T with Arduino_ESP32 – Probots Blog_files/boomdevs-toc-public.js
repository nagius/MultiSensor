(function($) {
    "use strict";

    $(document).ready(function() {
        let bdTocContentListLink = $(".bd_toc_content_list_item a"),
            bdTocContainerWithFixed = $(".scroll-to-fixed-fixed"),
            bdTocContainer = $(".bd_toc_container"),
            bdTocHeader = $(".bd_toc_header"),
            bd_toc_wrapper = $(".bd_toc_wrapper"),
            bd_toc_content = $(".bd_toc_content"),
            bd_toc_wrapper_height = bd_toc_wrapper.height(),
            bd_toc_content_height = $(".bd_toc_content").height(),
            bdTocContainerDataVal = bdTocContainer.data("fixedwidth");

        bdTocHeader.click(function() {
            bd_toc_content.slideToggle();


            $('.fit_content').css({
                width: "auto"
            });

            if ($(this).hasClass("active")) {
                $(this).removeClass("active");
                bdTocContainer.addClass("slide_left");
                headerTitleWidthHeight();
            } else {
                $(this).addClass("active");
                bdTocContainer.removeClass("slide_left");
                headerTitleRemoveWidthHeight();
            }
        });


        //add width to header title
        function headerTitleWidthHeight() {
            let headerTitleWidth = $(".bd_toc_header_title").width();
            let bdTocWrapperPadding = bd_toc_wrapper.attr("data-wrapperPadding");
            let bdTocHeaderPadding = bdTocHeader.attr("data-headerPadding");
            let bdTocWrapperPaddingValue = parseInt(bdTocWrapperPadding);
            let bdTocHeaderPaddingValue = parseInt(bdTocHeaderPadding);
            let totleHeaderTitleWidth = parseInt(headerTitleWidth + bdTocWrapperPaddingValue + bdTocHeaderPaddingValue + 3);

            if (bdTocContainer.hasClass("scroll-to-fixed-fixed")) {
                bdTocContainerWithFixed.css({
                    position: "fixed",
                    top: "0px",
                });
            } else {
                bdTocContainer.css(
                    "cssText",
                    "width: " + totleHeaderTitleWidth + "px !important;"
                );
            }
        }

        // remove width and height from header title
        function headerTitleRemoveWidthHeight() {
            bdTocContainer.css("width", 0 + "px");
            bd_toc_content.css("cssText", "height: " + bd_toc_content_height + "px");
        }

        //collapse button off
        if (handle.initial_view == "0") {
            bdTocHeader.click(function() {
                if (bdTocContainer.hasClass("scroll-to-fixed-fixed")) {
                    bdTocContainerWithFixed.css({
                        position: "fixed",
                        top: "0px",
                    });
                } else {
                    bdTocContainer.css(
                        "cssText",
                        "width: " + 100 + "% !important;",
                        "transition: all 0.5s ease-in-out;"
                    );
                }
                bd_toc_content.css("cssText", "display: block;");

                if (bdTocContainer.hasClass("slide_left")) {
                    headerTitleWidthHeight();
                } else {
                    $(".bd_toc_content_list").css(
                        "cssText",
                        "height: " + $(".bd_toc_content").height() + "px !important;"
                    );
                }
            });
            if (bdTocHeader.hasClass("active")) {
                bdTocHeader.removeClass("active");
            }
        }

        //collapse button on
        if (handle.initial_view == "1") {
            headerTitleRemoveWidthHeight();
        }

        function slidingTocContainer(width) {
            $(".bd_toc_container.scroll-to-fixed-fixed").css(
                "cssText",
                `z-index: 1000;
      position: fixed;
      transition: .1s;
      top: 0px;
      margin-left: 0px;
      ${sticky_mode_position.sticky_mode_position}: calc( 0% + ${width} ) !important`
            );
        }

        $(document).on(
            "click",
            ".bd_toc_content_list_item ul li .collaps-button",
            function() {
                $(this).parent("li").toggleClass("collapsed");
                $(this).parent("li").find("li").addClass("collapsed");
                bd_toc_content_height = $(".bd_toc_content ul").height();

                toggleContentWrapperHeight(bd_toc_content_height);
            }
        );

        bdTocContentListLink.on("click", function() {
            let bdTocContentListLinkHref = $(this).attr("href");
            location.replace(bdTocContentListLinkHref);
        });

        bdTocContentListLink.on("click", function() {
            if (screen.width < "1024") {
                if (bdTocContainer.hasClass("active")) {
                    slidingTocContainer("-" + bdTocContainerDataVal + "px");
                    bdTocContainer.removeClass("active");
                }
            }
        });

        function toggleContentWrapperHeight(height) {
            $(".bd_toc_container .bd_toc_content_list_item").css("height", height);
        }

        $(".layout_toggle_button").on("click", function() {
            if (bdTocContainer.hasClass("active")) {
                bdTocContainer.removeClass("active");
                slidingTocContainer("-" + bdTocContainerDataVal + "px");
            } else {
                bdTocContainer.addClass("active");
                slidingTocContainer(`0px`);
            }
        });
    });

    $(".bd_toc_content_list_item").onePageNav({
        currentClass: "current",
        scrollChange: function($currentListItem) {
            $($currentListItem).parents("li").addClass("active");
            $($currentListItem[0]).addClass("active");
        },
    });

    let count = $(".bd_toc_container").length;
    for (let i = 0; i < count; i++) {
        if (i !== 0) {
            $(".bd_toc_container").eq(1).remove();
        }
    }


    // if ($('.boomdevs_toc_shortcode_widget').length > 0) {
    //     $('.boomdevs_toc_shortcode_widget').append($('.bd_toc_container'));
    // }
})(jQuery);